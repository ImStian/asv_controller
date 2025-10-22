#!/usr/bin/env python3
import math
from typing import Optional
from collections import deque

import numpy as np
import yaml

plt = None  # Deferred matplotlib reference
MATPLOTLIB_IMPORT_ERROR: Optional[Exception] = None


def _ensure_matplotlib_loaded():
	"""Attempt to import matplotlib lazily, returning the first error seen."""
	global plt, MATPLOTLIB_IMPORT_ERROR
	if plt is not None or MATPLOTLIB_IMPORT_ERROR is not None:
		return MATPLOTLIB_IMPORT_ERROR
	try:
		import matplotlib.pyplot as _plt  # type: ignore
		plt = _plt
		return None
	except Exception as exc:  # pragma: no cover - environment dependent
		MATPLOTLIB_IMPORT_ERROR = exc
		plt = None
		return exc

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from .asv_controller_node import ControllerNode


def yaw_to_quaternion(yaw: float):
	"""Convert yaw angle to quaternion (x, y, z, w)."""
	half = 0.5 * yaw
	return (0.0, 0.0, math.sin(half), math.cos(half))


class AsvTowfishSim(Node):
	"""Minimal ASV + towfish dynamics using the controller's hull model."""

	def __init__(self):
		super().__init__('asv_towfish_sim')

		# Pull the same hull model the controller uses so dynamics stay consistent.
		dimensions = (1.20, 0.93, 0.20)
		x_g = -0.2
		time_constants = (1.0, 0.8, 1.2)
		current_vec = (0.0, 0.0)
		self.model = ControllerNode.asv_model(dimensions, x_g, time_constants, current_vec)

		rate_hz = float(self.declare_parameter('rate_hz', 100.0).get_parameter_value().double_value)
		self.dt = 1.0 / max(rate_hz, 1.0)

		# ASV states: eta = [x, y, psi], nu = [u, v, r]
		self.eta = np.array([0, 1.0, 0.0], dtype=float)
	
		self.nu = np.zeros(3, dtype=float)

		# Towfish simplified mass-spring in inertial frame.
		self.tow_pos = np.array([-3.5, 1.0], dtype=float)
		self.tow_vel = np.zeros(2, dtype=float)
		self.tether_length = float(self.declare_parameter('tether_length', 3.5).get_parameter_value().double_value)
		self.tow_mass = float(self.declare_parameter('tow_mass', 25.0).get_parameter_value().double_value)
		self.tow_stiffness = float(self.declare_parameter('tow_stiffness', 12.0).get_parameter_value().double_value)
		self.tow_damping = float(self.declare_parameter('tow_damping', 35.0).get_parameter_value().double_value)

		# Thruster commands (N)
		self.thrust_port: float = 0.0
		self.thrust_stbd: float = 0.0

		# Reference path for plotting
		path_param = self.declare_parameter('path_file', '').get_parameter_value().string_value
		pts = self._load_path_points(path_param)
		self.path_points = np.array(pts, dtype=float)
		self._update_path_geometry(self.path_points)

		# Real-time plotting setup (optional).
		self.enable_plot = bool(self.declare_parameter('enable_plot', True).get_parameter_value().bool_value)
		self.plot_stride = max(1, int(self.declare_parameter('plot_stride', 5).get_parameter_value().integer_value))
		self._history_len = max(10, int(self.declare_parameter('plot_history', 500).get_parameter_value().integer_value))
		self.show_s_point = bool(self.declare_parameter('show_s_point', True).get_parameter_value().bool_value)
		self._plot_counter = 0
		self._asv_history = deque(maxlen=self._history_len)
		self._tow_history = deque(maxlen=self._history_len)
		self._s_history = deque(maxlen=self._history_len)
		if self.enable_plot:
			err = _ensure_matplotlib_loaded()
			if plt is None:
				reason = f' ({err})' if err else ''
				self.get_logger().warning('matplotlib not available; disabling plotting.' + reason)
				self.enable_plot = False
		if self.enable_plot:
			plt.ion()
			self._fig, self._ax = plt.subplots()
			self._path_line = None
			if self.path_points is not None and len(self.path_points) > 1:
				self._path_line, = self._ax.plot(
					self.path_points[:, 0],
					self.path_points[:, 1],
					'k--',
					lw=1.0,
					label='Reference path',
				)
			self._asv_path_line, = self._ax.plot([], [], 'b-', lw=1.0, label='ASV path')
			self._tow_path_line, = self._ax.plot([], [], 'r-', lw=1.0, label='Towfish path')
			self._asv_scatter = self._ax.scatter([], [], c='blue', marker='o')
			self._tow_scatter = self._ax.scatter([], [], c='red', marker='x')
			self._s_scatter = self._ax.scatter([], [], c='green', marker='s', s=50, label='LOS point') if self.show_s_point else None
			self._ax.set_xlabel('East [m]')
			self._ax.set_ylabel('North [m]')
			self._ax.set_title('ASV and Towfish (NED frame)')
			self._ax.grid(True)
			self._ax.set_aspect('equal', 'box')
			self._ax.legend(loc='upper right')
			self._fig.canvas.draw()
			self._fig.canvas.flush_events()

		# I/O
		self.port_sub = self.create_subscription(
			Float64,
			'/model/blueboat/joint/motor_port_joint/cmd_thrust',
			self._port_cb,
			10,
		)
		self.stbd_sub = self.create_subscription(
			Float64,
			'/model/blueboat/joint/motor_stbd_joint/cmd_thrust',
			self._stbd_cb,
			10,
		)

		self.asv_pub = self.create_publisher(Odometry, '/model/blueboat/odometry', 10)
		self.tow_pub = self.create_publisher(Odometry, '/model/bluerov2_heavy/odometry', 10)

		self.last_time: Optional[float] = None
		self.timer = self.create_timer(self.dt, self._step)

	# ------------------------------------------------------------------
	def _port_cb(self, msg: Float64):
		self.thrust_port = float(msg.data)

	def _stbd_cb(self, msg: Float64):
		self.thrust_stbd = float(msg.data)

	# ------------------------------------------------------------------
	def _step(self):
		now = self.get_clock().now().nanoseconds * 1e-9
		if self.last_time is None:
			self.last_time = now
			return

		dt = max(now - self.last_time, 1e-4)
		self.last_time = now

		tau = self._thruster_to_body_forces()
		self._integrate_asv(tau, dt)
		self._integrate_towfish(dt)

		self._publish_asv(now)
		self._publish_towfish(now)
		self._update_plot()

	# ------------------------------------------------------------------
	def _thruster_to_body_forces(self) -> np.ndarray:
		beam = self.model['dimensions'][1]
		half_beam = 0.5 * beam
		surge = self.thrust_port + self.thrust_stbd
		yaw = half_beam * (self.thrust_port - self.thrust_stbd)
		return np.array([surge, 0.0, yaw], dtype=float)

	def _integrate_asv(self, tau: np.ndarray, dt: float):
		m = self.model['mass']
		j = self.model['inertia']
		x_g = self.model['xG']
		xu = self.model['Xu']
		yv = self.model['Yv']
		nr = self.model['Nr']
		dul = self.model['Dul']
		dvl = self.model['Dvl']
		drl = self.model['Drl']

		u, v, r = self.nu

		m11 = m + xu
		m22 = m + yv
		m23 = m * x_g
		m33 = j + nr

		M = np.array(
			[
				[m11, 0.0, 0.0],
				[0.0, m22, m23],
				[0.0, m23, m33],
			],
			dtype=float,
		)

		C = np.array(
			[
				[0.0, -m * r, 0.0],
				[m * r, 0.0, 0.0],
				[0.0, 0.0, 0.0],
			],
			dtype=float,
		)

		D = np.diag([dul, dvl, drl])

		rhs = tau - (C + D) @ self.nu
		nu_dot = np.linalg.solve(M, rhs)
		self.nu += nu_dot * dt

		psi = self.eta[2]
		c = math.cos(psi)
		s = math.sin(psi)
		J = np.array(
			[
				[c, -s, 0.0],
				[s, c, 0.0],
				[0.0, 0.0, 1.0],
			],
			dtype=float,
		)

		eta_dot = J @ self.nu
		self.eta += eta_dot * dt
		self.eta[2] = (self.eta[2] + math.pi) % (2.0 * math.pi) - math.pi

	def _integrate_towfish(self, dt: float):
		# Spring-damper forcing keeps the towfish near the tether anchor.
		anchor = self.eta[:2] - self.tether_length * np.array([
			math.cos(self.eta[2]),
			math.sin(self.eta[2]),
		])

		error = anchor - self.tow_pos
		accel = (self.tow_stiffness * error - self.tow_damping * self.tow_vel) / max(self.tow_mass, 1e-3)
		self.tow_vel += accel * dt
		self.tow_pos += self.tow_vel * dt

	# ------------------------------------------------------------------
	def _publish_asv(self, stamp_sec: float):
		msg = Odometry()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'map'
		msg.child_frame_id = 'blueboat/base_link'
		msg.pose.pose.position.x = float(self.eta[0])
		msg.pose.pose.position.y = float(self.eta[1])
		msg.pose.pose.position.z = 0.0
		qx, qy, qz, qw = yaw_to_quaternion(self.eta[2])
		msg.pose.pose.orientation.x = qx
		msg.pose.pose.orientation.y = qy
		msg.pose.pose.orientation.z = qz
		msg.pose.pose.orientation.w = qw
		msg.twist.twist.linear.x = float(self.nu[0])
		msg.twist.twist.linear.y = float(self.nu[1])
		msg.twist.twist.linear.z = 0.0
		msg.twist.twist.angular.z = float(self.nu[2])
		self.asv_pub.publish(msg)

	def _publish_towfish(self, stamp_sec: float):
		msg = Odometry()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'map'
		msg.child_frame_id = 'bluerov2_heavy/base_link'
		msg.pose.pose.position.x = float(self.tow_pos[0])
		msg.pose.pose.position.y = float(self.tow_pos[1])
		msg.pose.pose.position.z = 0.0

		tow_yaw = math.atan2(self.tow_vel[1], self.tow_vel[0]) if np.linalg.norm(self.tow_vel) > 1e-4 else self.eta[2]
		qx, qy, qz, qw = yaw_to_quaternion(tow_yaw)
		msg.pose.pose.orientation.x = qx
		msg.pose.pose.orientation.y = qy
		msg.pose.pose.orientation.z = qz
		msg.pose.pose.orientation.w = qw

		msg.twist.twist.linear.x = float(self.tow_vel[0])
		msg.twist.twist.linear.y = float(self.tow_vel[1])
		msg.twist.twist.linear.z = 0.0
		msg.twist.twist.angular.z = 0.0
		self.tow_pub.publish(msg)

	def _update_plot(self):
		if not self.enable_plot:
			return
		self._asv_history.append(self.eta[:2].copy())
		self._tow_history.append(self.tow_pos.copy())
		if self.show_s_point and self._segments_valid():
			s_point = self._compute_s_point()
			self._s_history.append(s_point)
		self._plot_counter += 1
		if self._plot_counter % self.plot_stride != 0:
			return
		try:
			if self._asv_history:
				asv_hist = np.array(self._asv_history)
				self._asv_path_line.set_data(asv_hist[:, 0], asv_hist[:, 1])
				self._asv_scatter.set_offsets([asv_hist[-1]])
			if self._tow_history:
				tow_hist = np.array(self._tow_history)
				self._tow_path_line.set_data(tow_hist[:, 0], tow_hist[:, 1])
				self._tow_scatter.set_offsets([tow_hist[-1]])
			if self.show_s_point and self._s_history and self._s_scatter is not None:
				s_hist = np.array(self._s_history)
				self._s_scatter.set_offsets([s_hist[-1]])
			self._ax.relim()
			self._ax.autoscale_view()
			self._fig.canvas.draw_idle()
			plt.pause(0.001)
		except Exception as exc:
			self.get_logger().warning(f'Plot update failed: {exc}')
			self.enable_plot = False

	def _segments_valid(self) -> bool:
		return self._segment_starts is not None and self._segment_vectors is not None and len(self._segment_starts) > 0

	def _compute_s_point(self) -> np.ndarray:
		"""Project the ASV position onto the closest point along the polyline path."""
		if not self._segments_valid():
			return self.eta[:2].copy()
		pos = self.eta[:2]
		best_point = self._segment_starts[0]
		best_dist = float('inf')
		for start, vec, length in zip(self._segment_starts, self._segment_vectors, self._segment_lengths):
			if length <= 1e-9:
				continue
			delta = pos - start
			t = float(np.dot(delta, vec) / (length * length))
			t = min(1.0, max(0.0, t))
			candidate = start + t * vec
			dist = float(np.linalg.norm(pos - candidate))
			if dist < best_dist:
				best_dist = dist
				best_point = candidate
		return best_point.copy()

	def _update_path_geometry(self, points: Optional[np.ndarray]):
		if points is None or len(points) < 2:
			self._segment_starts = None
			self._segment_vectors = None
			self._segment_lengths = None
			return
		pts = np.array(points, dtype=float)
		seg_vecs = pts[1:] - pts[:-1]
		seg_len = np.linalg.norm(seg_vecs, axis=1)
		mask = seg_len > 1e-6
		self._segment_starts = pts[:-1][mask]
		self._segment_vectors = seg_vecs[mask]
		self._segment_lengths = seg_len[mask]

	def _load_path_points(self, path_file: str) -> Optional[np.ndarray]:
		"""Load reference path waypoints for visualization."""
		if path_file:
			try:
				with open(path_file, 'r') as stream:
					data = yaml.safe_load(stream) or {}
					pts = data.get('waypoints')
					if isinstance(pts, list) and len(pts) >= 2:
						return np.array([[float(x), float(y)] for x, y in pts], dtype=float)
			except Exception as exc:  # pragma: no cover - file errors
				self.get_logger().warning(f'Failed to load path waypoints from {path_file}: {exc}')
		# Fallback to a simple straight path for context
		x_vals = np.linspace(-10.0, 40.0, 50)
		return np.stack([x_vals, np.zeros_like(x_vals)], axis=1)


def main(args=None):
	rclpy.init(args=args)
	node = AsvTowfishSim()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
