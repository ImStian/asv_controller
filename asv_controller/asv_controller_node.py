#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # Change back to Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import yaml
import math
from rclpy.parameter import Parameter
import numpy as np
from autograd import jacobian # for los
import os

import logging
import functools
import inspect
import json
from datetime import datetime



class ControllerNode(Node):
    # Initialize Logger
    logger = logging.getLogger("sim_debug")
    logger.setLevel(logging.DEBUG)
    # Ensure a dedicated FileHandler is attached so logs go to sim_debug.log even if
    # the root logger is configured elsewhere (common in ROS setups).
    if not logger.handlers:
        fh = logging.FileHandler("sim_debug.log", mode="a")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter("%(message)s"))
        logger.addHandler(fh)
    # Avoid propagating to root handlers to prevent duplicates
    logger.propagate = False

    def log_variables(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            # Get argument names
            sig = inspect.signature(func)
            bound_args = sig.bind(*args, **kwargs)
            bound_args.apply_defaults()

            # Prepare dictionary of input names -> values
            inputs = {name: repr(value) for name, value in bound_args.arguments.items()}

            # Call function
            result = func(*args, **kwargs)

            # If result is a dict or tuple, add names
            output_info = {}
            if isinstance(result, dict):
                output_info = {k: repr(v) for k, v in result.items()}
            elif isinstance(result, (tuple, list)):
                output_info = {f"output_{i}": repr(v) for i, v in enumerate(result)}
            else:
                output_info = {"return": repr(result)}

            # Log JSON line
            record = {
                "timestamp": datetime.now().isoformat(),
                "function": func.__name__,
                "inputs": inputs,
                "outputs": output_info,
            }

            ControllerNode.logger.debug(json.dumps(record))
            return result

        return wrapper   



    def __init__(self):
        super().__init__('controller_node')

        # Parameters for LOS
        self.params = {
            "los": 20.0,   # Lookahead distance for LOS (m)
            "U": 5.0,      # Desired speed [m/s]
            "k": 0.3       # Parameter update gain
        }  
        # Default straight-line path (will be replaced if a waypoint YAML is available)
        self.path_fcn = lambda s: np.array([s, 0.0])  # path function: straight line along x-axis

        # Try loading a default waypoint file (circle) placed in the package root
        try:
            pkg_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
            yaml_path = os.path.join(pkg_dir, 'circle_radius_3m.yaml')
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as fh:
                    data = yaml.safe_load(fh)
                if data and 'waypoints' in data and isinstance(data['waypoints'], list):
                    pts = [(float(x), float(y)) for x, y in data['waypoints']]
                    f, fdot, total = self.build_piecewise_path(pts)
                    self.path_fcn = lambda s, _f=f: np.array(_f(s), dtype=float)
                    self.path_fcn_dot = lambda s, _fd=fdot: np.array(_fd(s), dtype=float)
                    self._wp_points = pts
                    self._wp_total = total
                    self.use_waypoints = True
                    self.get_logger().info(f'Loaded default waypoints from {yaml_path} (total length {total:.2f} m)')
        except Exception as e:
            self.get_logger().warn(f'Could not load default waypoint file: {e}')
        
        # Initialize the state vector x_i
        self.s = 0.0  # Initial path parameter
        self.zeta = np.zeros(7)  # Adjust size based on your parameter vector
        self._prev_control_time = 0.0
        

        # Parameters for the controller
        self.m_virtual = 225.0  # Virtual mass of the ASV (kg) (Keeping close to actual mass)
        self.k_v = 1.0  # Velocity control gain
        self.k_a = 0.5  # Acceleration control gain
        self.L = 3.5  # Length of the tether (m)
        self.epsilon = 0.5  # Small positive constant for adaptive controller
        self.k_psi = 1.0  # Heading control gain
        self.k_r = 1.0  # Yaw rate control gain
        self.declare_parameter('heading_mode', 'LOS') # 'LOS' or 'Path'
        # Thruster safety parameters
        self.declare_parameter('thruster_sign_port', 1.0)
        self.declare_parameter('thruster_sign_stbd', 1.0)
        self.declare_parameter('max_thrust', 50.0)

        # System Model (See seperate section in this file for functions)
        dimensions = (1.20, 0.93, 0.20) # Dimensions of the ASV [length, width, draft] in meters
        xG = -0.2 # Center of Drag
        tc = (1.0, 0.8, 1.2) # Time Constants [surge, sway, yaw]
        V_current = (0.0, 0.0)  # [m/s] Just assume no current for now
        self.mdl = self.asv_model(dimensions, xG, tc, V_current)

        # Subscriptions for ASV and ROV odometry
        self.blueboat_odom_sub = self.create_subscription(
            Odometry,
            '/model/blueboat/odometry',
            self.blueboat_odom_callback,
            10)

        self.rov_odom_sub = self.create_subscription(
            Odometry,
            '/model/bluerov2_heavy/odometry', 
            self.rov_odom_callback,
            10)

        # Store latest odometry
        self.blueboat_odom = None
        self.rov_odom = None

        # Publishers for Blueboat thrust (0-100%)
        self.port_thrust_pub = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_port_joint/cmd_thrust',
            10)
        self.stbd_thrust_pub = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_stbd_joint/cmd_thrust',
            10)
        self.get_logger().info('Controller node started.')

    #########################   COORDINATE CONVERSION  ################################
    def lla_to_enu(self, lat, lon, ref_lat, ref_lon, ref_alt):
        """
        Convert latitude/longitude (degrees) to local ENU coordinates (meters) using a reference origin.
        Assumes small area (flat earth approximation).
        """
        # WGS84 constants
        a = 6378137.0  # Equatorial radius
        f = 1/298.257223563
        b = a * (1 - f)
        # Convert degrees to radians
        lat = math.radians(lat)
        lon = math.radians(lon)
        ref_lat = math.radians(ref_lat)
        ref_lon = math.radians(ref_lon)
        # Radius of curvature in prime vertical
        N = a / math.sqrt(1 - (2*f - f**2) * (math.sin(lat)**2))
        N_ref = a / math.sqrt(1 - (2*f - f**2) * (math.sin(ref_lat)**2))
        # ECEF coordinates
        x = (N + 0) * math.cos(lat) * math.cos(lon)
        y = (N + 0) * math.cos(lat) * math.sin(lon)
        z = (b**2/a**2 * N + 0) * math.sin(lat)
        x0 = (N_ref + 0) * math.cos(ref_lat) * math.cos(ref_lon)
        y0 = (N_ref + 0) * math.cos(ref_lat) * math.sin(ref_lon)
        z0 = (b**2/a**2 * N_ref + 0) * math.sin(ref_lat)
        # ENU conversion
        dx = x - x0
        dy = y - y0
        dz = z - z0
        t = -math.sin(ref_lon)*dx + math.cos(ref_lon)*dy
        e = -math.sin(ref_lon)*(x-x0) + math.cos(ref_lon)*(y-y0)
        n = -math.sin(ref_lat)*math.cos(ref_lon)*(x-x0) - math.sin(ref_lat)*math.sin(ref_lon)*(y-y0) + math.cos(ref_lat)*(z-z0)
        return [e, n]

    def enu_to_lla(self, e, n, ref_lat, ref_lon, ref_alt):
        """Convert local ENU coordinates back to LLA."""
        ref_lat = math.radians(ref_lat)
        ref_lon = math.radians(ref_lon)
        
        # WGS84 parameters
        a = 6378137.0  # Equatorial radius
        f = 1/298.257223563
        b = a * (1 - f)
        
        # Radius of curvature
        N_ref = a / math.sqrt(1 - (2*f - f**2) * (math.sin(ref_lat)**2))
        
        # ECEF of reference point
        x0 = (N_ref + ref_alt) * math.cos(ref_lat) * math.cos(ref_lon)
        y0 = (N_ref + ref_alt) * math.cos(ref_lat) * math.sin(ref_lon)
        z0 = (b**2/a**2 * N_ref + ref_alt) * math.sin(ref_lat)
        
        # ENU to ECEF
        dx = -n * math.sin(ref_lat) * math.cos(ref_lon) - e * math.sin(ref_lon)
        dy = -n * math.sin(ref_lat) * math.sin(ref_lon) + e * math.cos(ref_lon)
        dz = n * math.cos(ref_lat)
        
        x = x0 + dx
        y = y0 + dy
        z = z0 + dz
        
        # ECEF to LLA
        p = math.sqrt(x**2 + y**2)
        lat = math.atan2(z, p * (1 - f))
        lon = math.atan2(y, x)
        
        return math.degrees(lat), math.degrees(lon)

    ####################### MEASUREMENTS ################################
    @log_variables
    def blueboat_odom_callback(self, msg):
        """Handle odometry updates from the ASV.
            -Odometry.
            -Position in ENU
            -Velocity and Angular Velocity in the body frame.
            -Heading (yaw) in radians.
        """
        self.blueboat_odom = msg
        
        # Get position in ENU
        pos = msg.pose.pose.position
        
        # Log position information
        self.get_logger().info(f'ASV Position - ENU: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')

        # raw linear velocity from odom message (m/s)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.blueboat_vel = [vx, vy, vz]

        # Angular velocity (rad/s) from odometry message
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z
        self.blueboat_angular_vel = [wx, wy, wz]

        # Compute and store heading (yaw) in radians from the odometry quaternion
        self.blueboat_heading = math.atan2(
            2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
            1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        )
        self.get_logger().info(f'ASV heading (rad): {self.blueboat_heading:.3f}')

        # Only proceed with control if we have all necessary data
        if self.blueboat_odom is not None and self.rov_odom is not None:
            self.control_law()

    @log_variables
    def rov_odom_callback(self, msg):
        """Handle odometry updates from the ROV.
            -Odometry
            -Position in ENU
        """
        self.rov_odom = msg
        
        # Get position in ENU
        pos = msg.pose.pose.position
        # Log ROV position
        self.get_logger().info(f'ROV Position - ENU: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')

    @log_variables
    def calculate_relative_position(self):
        """Calculate relative position and angle between ASV and ROV."""
        if not hasattr(self, 'blueboat_odom') or not hasattr(self, 'rov_odom'):
            return None, None, None

        asv_pos = self.blueboat_odom.pose.pose.position
        rov_pos = self.rov_odom.pose.pose.position

        # Calculate relative vector
        dx = rov_pos.x - asv_pos.x
        dy = rov_pos.y - asv_pos.y
        dz = rov_pos.z - asv_pos.z

        # Calculate distance and angle
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        angle = math.atan2(dy, dx)  # Angle in radians from ASV to ROV

        # Try analytic derivative using odometry linear velocities (preferred)
        theta_dot = None
        try:
            vx_asv = float(self.blueboat_odom.twist.twist.linear.x)
            vy_asv = float(self.blueboat_odom.twist.twist.linear.y)
            vx_rov = float(self.rov_odom.twist.twist.linear.x)
            vy_rov = float(self.rov_odom.twist.twist.linear.y)

            dx_dot = vx_rov - vx_asv
            dy_dot = vy_rov - vy_asv
            r2 = dx*dx + dy*dy
            if r2 > 1e-8:
                theta_dot = (dx * dy_dot - dy * dx_dot) / r2
            else:
                theta_dot = 0.0
        except Exception:
            theta_dot = None

        # Fallback: finite-difference on angle over time if analytic not available
        if theta_dot is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if hasattr(self, '_prev_angle') and hasattr(self, '_prev_time'):
                dt = now - self._prev_time
                if dt > 1e-6:
                    dtheta = angle - self._prev_angle
                    # unwrap to [-pi, pi]
                    dtheta = (dtheta + math.pi) % (2*math.pi) - math.pi
                    theta_dot = dtheta / dt
                else:
                    theta_dot = 0.0
            else:
                theta_dot = 0.0
            # store for next iteration
            self._prev_angle = angle
            self._prev_time = now

        return distance, angle, theta_dot
    
    ######################## MRAC CONTROLLER ##############################
    @log_variables
    def control_law(self):
        "Compute the control forces and state derivaties for the ASV towing controller"

        # Unpack the state
        p0 = self.blueboat_odom.pose.pose.position # position in ENU
        psi = self.blueboat_heading # heading
        v0 = self.blueboat_vel # Velocity of the ASV in the body frame
        r = self.blueboat_angular_vel[2] # Yaw rate (rad/s)
        q = np.array([p0.x, p0.y, psi]) # Full ASV state [x, y, psi]
        q_dot = np.array([v0[0], v0[1], r]) # ASV velocity state [u, v, r]
        
        rel = self.calculate_relative_position()
        if rel is None or rel[0] is None:
             return  # not enough data yet
        _ , theta, theta_dot = rel # Angle and Angular velocity of the pendulum
        if theta_dot is None:
            theta_dot = 0.0  # fallback if not available



        # Get current time in seconds
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._prev_control_time is not None:
            dt = now - self._prev_control_time
        else:
            dt = 1e-6
        self._prev_control_time = now

        # Use class member states
        s = self.s  # Path parameter
        zeta = self.zeta  # Parameter estimates vector


        # Headway computation
        Gamma = np.array([np.cos(theta), np.sin(theta)])
        dGamma = np.array([-np.sin(theta), np.cos(theta)])
        # Convert p0 to a 2D numpy array
        p = np.array([p0.x, p0.y]) + self.epsilon * Gamma  # Position of the pendulum mass
        v = np.array(v0[:2]) + self.epsilon * theta_dot * dGamma  # Velocity of the pendulum mass

        # Compute Reference Velocity (requires LOS)
        v_ref, s_dot = self.line_of_sight(p, s, self.path_fcn, self.params)

        # Approximate acceleration of the pendulum mass
        v_ref_plus, _ = self.line_of_sight(p + dt * v, self.s + dt * s_dot, self.path_fcn, self.params)
        # safe finite difference for v_ref_dot
        if dt <= 1e-6:
            v_ref_dot = np.zeros_like(v_ref, dtype=float)
        else:
            v_ref_plus, _ = self.line_of_sight(p + dt * v, self.s + dt * s_dot, self.path_fcn, self.params)
            v_ref_dot = (v_ref_plus - v_ref) / dt

        # Adaption law
        u_p, zeta_dot = self.pendulum_adaptive_controller(
            np.concatenate(([theta, theta_dot], [p0.x, p0.y], v0[:2])),
            zeta,
            v_ref,
            v_ref_dot,
            self.L,
            self.epsilon,
            self.k_v,
            self.k_a
        )

        # Heading control to align ASV with the pendulum direction
        if self.get_parameter('heading_mode').get_parameter_value().string_value == 'LOS':
            def path_angle(_s):
                dp = self.numerical_derivative(self.path_fcn, _s)
                return math.atan2(dp[1], dp[0])
            
            psi_ref = path_angle(self.s)
            r_ref = self.numerical_derivative(path_angle, self.s) * s_dot

        elif self.get_parameter('heading_mode').get_parameter_value().string_value == 'Path':
            psi_ref = np.arctan2(v_ref[1], v_ref[0])
            # Approximate the derivative
            psi_ref_plus = np.arctan2(v_ref_dot[1], v_ref_dot[0])
            r_ref = (psi_ref_plus - psi_ref) / dt

        else:
            self.get_logger().error("Invalid heading_mode parameter. Use 'LOS' or 'Path'.")
            return
        
        # Heading Control Law
        psi_error = ((psi - psi_ref + np.pi) % (2 * np.pi)) - np.pi  # Wrap to [-pi, pi]
        u_r = -self.k_psi * psi_error - self.k_r * (r - r_ref)

        # Create control input vector - Fix the inhomogeneous shape issue
        u = np.array([*u_p, u_r])  # Unpack u_p components and append u_r
        # Alternative: u = np.concatenate([u_p, [u_r]])
            
        tau = self.asv_virtual_mass_controller(q, q_dot, psi, self.mdl, u, self.m_virtual)
        T_L, T_R = self.asv_thrust_allocation(tau, self.mdl)


        # Update states with time integration
        if dt > 0:
            self.s += s_dot * dt  # Update path parameter
            self.s = np.clip(self.s, 0.0, self._wp_total)
            self.zeta += zeta_dot * dt  # Update parameter estimates

        # Publish thrust commands
        try:
            # Apply configurable sign and clamp to avoid extreme commands that flip the vehicle
            sign_port = float(self.get_parameter('thruster_sign_port').get_parameter_value().double_value)
            sign_stbd = float(self.get_parameter('thruster_sign_stbd').get_parameter_value().double_value)
            max_thrust = float(self.get_parameter('max_thrust').get_parameter_value().double_value)

            T_L_pub = float(np.clip(T_L * sign_port, -max_thrust, max_thrust))
            T_R_pub = float(np.clip(T_R * sign_stbd, -max_thrust, max_thrust))

            # Publish and log diagnostics
            self.port_thrust_pub.publish(Float64(data=T_L_pub))
            self.stbd_thrust_pub.publish(Float64(data=T_R_pub))
            self.get_logger().info('Thrust published (raw T_L: {:.3f}, raw T_R: {:.3f}) -> published (T_L: {:.3f}, T_R: {:.3f})'.format(T_L, T_R, T_L_pub, T_R_pub))
        except Exception as e:
            self.get_logger().error(f'Failed to publish thrust: {e}')
        
        return tau


    @log_variables
    def line_of_sight(self, p, s, path_fcn, params):
        # Unpack parameters
        U = params["U"]
        los = params["los"]
        k = params["k"]

        # Compute the path point
        p_path = path_fcn(s)
        # Try to use autograd jacobian if the path function supports it,
        # otherwise fall back to a numerical derivative. Converting to
        # a plain numpy float array prevents autograd ArrayBox values
        # from propagating into later numpy operations which can cause
        # inf/nan results.
        p_path_dot = np.array(self.numerical_derivative(path_fcn, s), dtype=float).reshape(-1)
        theta_path = math.atan2(p_path_dot[1], p_path_dot[0])
        R_path = np.array([[np.cos(theta_path), -np.sin(theta_path)], 
                           [np.sin(theta_path), np.cos(theta_path)]])
        delta_norm = np.linalg.norm(p_path_dot)

        # Path-following error:
        e = R_path.T @ (p - p_path)
        e_x = float(e[0])
        e_y = float(e[1])
        D = math.sqrt(float(los**2 + e_y**2))

        # If the path derivative is (near) zero we cannot update the
        # path parameter reliably — avoid producing inf/nan by
        # returning a safe, zero update and a LOS velocity of zero.
        if delta_norm <= 1e-8 or not np.isfinite(delta_norm):
            try:
                self.get_logger().warning('line_of_sight: path derivative nearly zero or non-finite; setting s_dot=0')
            except Exception:
                pass
            v_LOS = np.zeros(2, dtype=float)
            s_dot = 0.0
            return v_LOS, s_dot

        # LOS guidance law
        v_LOS = (U / D) * (R_path @ np.array([los, -e_y], dtype=float))

        # Path parameter update (safe division since delta_norm checked)
        s_dot = (U / float(delta_norm)) * (float(los) / float(D) * k * float(self.saturation(e_x)))

        return np.array(v_LOS, dtype=float), float(s_dot)

    @log_variables
    def pendulum_adaptive_controller(self, x, zeta, v_ref, v_ref_dot, L, epsilon, k_v, k_a):
        """Adaptive controller for the pendulum dynamics."""
        # Unpack states
        theta = x[0]
        theta_dot = x[1]
        v0 = x[4:6]

        Gamma = np.array([np.cos(theta), np.sin(theta)])
        dGamma = np.array([-np.sin(theta), np.cos(theta)])

        # Ensure v0 and v1 are 2D vectors
        v0 = np.array(v0).reshape(2)
        v = np.array(v0) + epsilon * L * theta_dot * dGamma
        v_err = v - v_ref

        v1 = np.array(v0) + L * theta_dot * dGamma
        J = np.outer(dGamma, dGamma)

        # Compute terms ensuring consistent dimensions
        term1 = L * theta_dot**2 * Gamma
        term2 = -theta_dot / (2 * (epsilon - 1)) * ((np.outer(Gamma, dGamma) + np.outer(dGamma, Gamma)) @ v_err)
        term3 = -J @ (v_ref_dot - k_v * v_err) / (epsilon - 1)

        # Stack columns ensuring all have shape (2, n) where n matches zeta dimension (7)
        Y = np.column_stack([
            (term1 + term2 + term3).reshape(2, 1),  # shape: (2,1)
            v0.reshape(2, 1),                       # shape: (2,1)
            (L * theta_dot * dGamma).reshape(2, 1), # shape: (2,1)
            (k_v * v_err - v_ref_dot).reshape(2, 1),# shape: (2,1)
            J @ v1.reshape(2, 1),                   # shape: (2,1)
            J,                                      # shape: (2,2)
        ])

        # Verify dimensions
        if Y.shape[1] != len(zeta):
            self.get_logger().error(f"Dimension mismatch: Y shape {Y.shape}, zeta length {len(zeta)}")
            raise ValueError(f"Y columns ({Y.shape[1]}) must match zeta length ({len(zeta)})")

        # Control force
        u = -Y @ zeta

        # Adaption law
        zeta_dot = k_a * (Y.T @ v_err)

        return u, zeta_dot

    @log_variables
    def pendulum_parameter_vector(self, m0, m, c0, c, epsilon, V_c=None):
        """Constructs the parameter vector for the adaptive controller of a pendulum system."""
        zeta = np.array([
            m * (1 - epsilon) - m0 * epsilon,
            -(c + c0),
            -c,
            (c + c0) * V_c,
            c * (1 + m0 + epsilon / (m * (epsilon -1))),
            -c * (1 + m0 + epsilon / (m * (epsilon -1))) * V_c,
            m + m0
        ]).T

        return zeta

    @log_variables
    def asv_virtual_mass_controller(self, q, q_dot, psi, mdl, u, m_virtual):
        """Virtual mass controller (+ Thrust Allocation) for underactuated ASVs."""
        # Unpack states - modified to handle 3-element input
        u_px, u_py = u[0:2]  # First two elements are planar force
        u_r = u[2]           # Last element is rotational

        # Step1: Converting desired planer force to surge direction
        F_u, psi_d, phi = self.underactuated_transform(np.array([u_px, u_py]), psi)

        # Step2: Desired body-frame force vector
        tau_d = np.array([F_u, 0.0, u_r])

        # Step3: Compute mass and damping terms
        M, b = self.mass_Coriolis(mdl, q, q_dot)

        # Step4: Virtual mass dynamic compentation
        tau = (M @ tau_d) / m_virtual - b
        tau[1] = 0.0  # No sway force

        return tau

    @log_variables
    def asv_thrust_allocation(self, tau, mdl):
        W = mdl["dimensions"][1]
        l = W / 2.0 # Distance from centerline to each thruster

        # Allocation matrix B maps [T_L, T_R] → [tau_u, tau_v, tau_r]
        B = np.array([
            [1.0, 1.0],   # surge (sum of thrusters)
            [0.0, 0.0],   # sway (no actuation)
            [ l , -l ]    # yaw (moment arm)
        ])

        # Solve for thruster forces (least-squares in case overdetermined)
        T = np.linalg.pinv(B) @ tau
        T_L, T_R = float(T[0]), float(T[1])

        return T_L, T_R
    
    # --- Helpers ---

    @log_variables
    def numerical_derivative(self, f, x, h=1e-6):
        """Simple central difference derivative for scalar x."""
        return (np.array(f(x + h)) - np.array(f(x - h))) / (2 * h)

    def build_piecewise_path(self, points):
        """Builds a piecewise-linear path function and analytic derivative from a list of 2D points.

        points: list of (x,y) pairs
        Returns: (path_fcn, path_fcn_dot, total_length)
        path_fcn(s) returns position at path distance s along the polyline (clamped)
        path_fcn_dot(s) returns the local tangent (dx/ds, dy/ds) (unit vector along segment)
        """
        pts = np.array(points, dtype=float)
        if pts.shape[0] < 2:
            raise ValueError("Need at least two waypoints to build a path")

        # segment vectors and lengths
        seg = pts[1:] - pts[:-1]
        seg_len = np.linalg.norm(seg, axis=1)
        # guard zero-length segments
        seg_len[seg_len <= 1e-8] = 1e-8
        cum = np.concatenate(([0.0], np.cumsum(seg_len)))
        total = cum[-1]

        def path_fcn_s(s):
            s_clamped = float(np.clip(s, 0.0, total))
            # find segment index
            idx = int(np.searchsorted(cum, s_clamped) - 1)
            if idx < 0:
                idx = 0
            if idx >= len(seg):
                return pts[-1].copy()
            ds = s_clamped - cum[idx]
            t = ds / seg_len[idx]
            return (1.0 - t) * pts[idx] + t * pts[idx + 1]

        def path_fcn_dot_s(s):
            # return unit tangent along segment containing s
            s_clamped = float(np.clip(s, 0.0, total))
            idx = int(np.searchsorted(cum, s_clamped) - 1)
            if idx < 0:
                idx = 0
            if idx >= len(seg):
                v = seg[-1]
            else:
                v = seg[idx]
            norm = np.linalg.norm(v)
            if norm <= 1e-8:
                return np.array([1.0, 0.0])
            return v / norm

        return path_fcn_s, path_fcn_dot_s, total

    @log_variables
    def waypoints_callback(self, msg):
        """Accept Float32MultiArray messages containing either [x,y] pairs (2*N) or [x,y,z] triplets (3*N).
        Updates the internal path function used by LOS.
        """
        data = list(msg.data)
        if len(data) == 0:
            self.get_logger().warn('Received empty waypoint message')
            return

        if len(data) % 2 == 0 and len(data) % 3 != 0:
            # interpret as x,y pairs
            pts = [(data[i], data[i+1]) for i in range(0, len(data), 2)]
        elif len(data) % 3 == 0:
            # interpret as x,y,z triplets, drop z
            pts = [(data[i], data[i+1]) for i in range(0, len(data), 3)]
        else:
            self.get_logger().warn('Waypoint array length not divisible by 2 or 3; ignoring')
            return

        try:
            f, fdot, total = self.build_piecewise_path(pts)
        except Exception as e:
            self.get_logger().error(f'Failed to build path from waypoints: {e}')
            return

        # store helper info
        self._wp_points = pts
        self._wp_total = total
        self.use_waypoints = True

        # Set path_fcn to use the new piecewise function and provide analytic derivative when possible
        def path_wrapper(s):
            return np.array(f(s), dtype=float)

        def path_wrapper_dot(s):
            return np.array(fdot(s), dtype=float)

        # attach analytic derivative for potential autograd use
        self.path_fcn = path_wrapper
        self.path_fcn_dot = path_wrapper_dot
        self.get_logger().info(f'Waypoints received: {pts}; total path length: {total:.2f}')

    @log_variables
    def saturation(self, x):
        """Implements a smooth saturation function that maps R -> [-1,1]"""
        return x / np.sqrt(1 + x**2)

    @log_variables
    def underactuated_transform(self, u_p, psi):
        """Implements Josef's rule for Virtual mass for underactuated ASVs."""
        ux, uy = u_p
        norm_u = math.hypot(ux, uy)
        if norm_u < 1e-6:
            return 0.0, psi, 0.0

        phi = math.atan2(uy, ux)
        if math.cos(phi - psi) >= 0:
            psi_d = phi
            F_u = norm_u
        else:
            psi_d = ((phi + np.pi) + np.pi) % (2*np.pi) - np.pi
            F_u = -norm_u
        return F_u, psi_d, phi

    @log_variables
    def mass_Coriolis(self, mdl, q, q_dot):
        """
        Compute the mass matrix M and damping vector b for the ASV model.
        
        Args:
            mdl (dict): Dictionary containing ASV model parameters
            q (np.ndarray): State vector [x, y, psi]
            q_dot (np.ndarray): Velocity vector [u, v, r]
            
        Returns:
            tuple: (M, b) where M is mass matrix and b is damping vector
        """
        psi = q[2]
        u, v, r = q_dot  # body-frame velocities

        m = mdl["mass"]
        J = mdl["inertia"]
        Xu, Yv, Nr, Yr = mdl["Xu"], mdl["Yv"], mdl["Nr"], mdl["Yr"]
        Dul, Dvl, Drl = mdl["Dul"], mdl["Dvl"], mdl["Drl"]

        # Mass matrix (simplified rigid-body + added mass approximation)
        M = np.array([
            [m + Xu, 0, 0],
            [0, m + Yv, m * mdl["xG"]],
            [0, m * mdl["xG"], J + Nr]
        ])

        # Coriolis + damping term (simplified)
        C = np.array([
            [0, -m * r, 0],
            [m * r, 0, 0],
            [0, 0, 0]
        ])

        D = np.diag([Dul, Dvl, Drl])
        b = (C + D) @ q_dot

        return M, b
    

 

    ####################### Model ################################
    @staticmethod
    def asv_model(dimensions, xG, time_constants, V_current):
        """Create a simple dynamic model of the ASV based on its dimensions and time constants."""
        # ASV dynamic model parameters
        L, W, B = dimensions
        rho_water = 1025 # Density of water (kg/m^3)

        m = L*W*B*rho_water # Mass (kg)
        J = m*(L**2 + W**2)/12 # Moment of inertia (kg*m^2) Assuming rectangular prism
        Xu = 0.5*m # Surge linear drag
        Yv = 1.5*m # Sway linear drag
        Nr = 1.7*J # Yaw linear drag
        Yr = 0.0 # Cross-coupling drag

        # Damping coefficients
        Tu, Tv, Tr = time_constants
        Dul = (m + Xu) / Tu # Surge damping
        Dvl = (m + Yv) / Tv # Sway damping
        Drl = (J + Nr) / Tr # Yaw damping

        # Quadratic damping
        Duq = 0.0 # Surge quadratic drag
        Dvq = 0.0 # Sway quadratic drag
        Drq = 0.0 # Yaw quadratic drag

        # Packaging model into dictionary
        mdl = {
            "dimensions": (L, W, B),
            "xG": xG,
            "mass": m,
            "inertia": J,
            "Xu": Xu,
            "Yv": Yv,
            "Nr": Nr,
            "Yr": Yr,
            "Dul": Dul,
            "Dvl": Dvl,
            "Drl": Drl,
            "Duq": Duq,
            "Dvq": Dvq,
            "Drq": Drq,
            "rho_water": rho_water,
            "Vfunc": V_current
        }
        return mdl



def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()