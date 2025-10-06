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

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        # Declare parameters for all controller/system gains
        self.declare_parameter('U', 1.0)
        self.declare_parameter('Delta', 5.0)
        self.declare_parameter('k', 0.5)
        self.declare_parameter('L', 3.5)
        self.declare_parameter('epsilon', 0.5)
        self.declare_parameter('k_v', 1.0)
        self.declare_parameter('k_a', 0.1)
        self.declare_parameter('m_virtual', 1.0)
        self.declare_parameter('path_file', '')

        self.declare_parameter('base_thrust', 20.0)
        self.declare_parameter('turn_gain', 80.0)
        # Sign flip for turn mapping in case thruster geometry requires inversion
        self.declare_parameter('turn_sign', 1.0)
        # Scale factor to convert controller forward command (asv_cmd) to thrust percent
        self.declare_parameter('thrust_scale', 10.0)
        # Heading PD gains (align with Julia controller)
        self.declare_parameter('k_psi', 1.0)
        self.declare_parameter('k_r', 0.5)

        # Subscribe to Blueboat GPS and ROV odometry
        self.blueboat_gps_sub = self.create_subscription(
            NavSatFix,
            '/model/blueboat/navsat',
            self.blueboat_gps_callback,
            10)
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

        # Store latest odometry
        self.blueboat_odom = None
        self.rov_odom = None

        # Path following state
        self.s = 0.0
        # whether s was initialized to the closest point on the path
        self.s_initialized = False
        self.path = None

        # Reference origin for ENU conversion (lat, lon, alt)
        self.declare_parameter('ref_lat', 0.0)
        self.declare_parameter('ref_lon', 0.0)
        self.declare_parameter('ref_alt', 0.0)
        self.ref_lat = self.get_parameter('ref_lat').get_parameter_value().double_value
        self.ref_lon = self.get_parameter('ref_lon').get_parameter_value().double_value
        self.ref_alt = self.get_parameter('ref_alt').get_parameter_value().double_value

        self.enu_path = None
        self.load_path()
        # Adaptive controller parameter estimates (persistent)
        # NUM_ZETA = 9 (matches Julia implementation)
        self.zeta = np.zeros(9)

        # store previous LOS output for optional derivative estimates
        self.prev_los = None

        # run control loop at 10 Hz even if callbacks are missed
        self.control_timer = self.create_timer(0.1, self.control_step)

    def load_path(self):
        """
        Loads waypoints from a YAML file. The file should look like:
        waypoints:
          - [lat1, lon1]
          - [lat2, lon2]
        Waypoints are latitude/longitude in degrees. They are converted to local ENU coordinates (meters)
        using the reference origin (ref_lat, ref_lon, ref_alt).
        """
        path_file = self.get_parameter('path_file').get_parameter_value().string_value
        if path_file:
            try:
                with open(path_file, 'r') as f:
                    data = yaml.safe_load(f)

                # Support two YAML formats:
                # 1) Geographic LLA waypoints (existing):
                #    waypoints:
                #      - [lat1, lon1]
                # 2) Local ENU waypoints (new):
                #    waypoints_enu:
                #      - [x1, y1]
                if isinstance(data, dict) and 'waypoints_enu' in data:
                    self.enu_path = data['waypoints_enu']
                    self.path = None
                    self.get_logger().info(f'Loaded ENU path from {path_file} (meters)')
                elif isinstance(data, dict) and 'waypoints' in data:
                    self.path = data['waypoints']  # expects list of [lat, lon]
                    # Convert lat/lon to ENU using reference origin
                    self.enu_path = [self.lla_to_enu(lat, lon, self.ref_lat, self.ref_lon, self.ref_alt) for lat, lon in self.path]
                    self.get_logger().info(f'Loaded path from {path_file} (lat/lon, converted to ENU)')
                else:
                    raise ValueError('YAML file must contain either "waypoints" or "waypoints_enu"')

            except Exception as e:
                self.get_logger().error(f'Failed to load path: {e}')
                # No fallback: require explicit path_file. Clear any loaded path.
                self.path = None
                self.enu_path = None
        else:
            self.path = None
            self.enu_path = None

    def find_closest_s(self, p):
        """Find the path parameter s (float) closest to point p=[x,y].
        Returns s in [0, len(path)-1]. Works with self.enu_path if available.
        Uses projection onto path segments.
        """
        path = self.enu_path if self.enu_path and len(self.enu_path) >= 2 else None
        if not path:
            # default straight line
            return p[0]

        best_s = 0.0
        best_d2 = float('inf')
        for i in range(len(path)-1):
            a = np.array(path[i])
            b = np.array(path[i+1])
            ap = np.array(p) - a
            ab = b - a
            ab_len2 = np.dot(ab, ab)
            if ab_len2 == 0:
                t = 0.0
            else:
                t = np.dot(ap, ab) / ab_len2
                t = max(0.0, min(1.0, t))
            proj = a + t * ab
            d2 = np.sum((proj - np.array(p))**2)
            s_val = i + t
            if d2 < best_d2:
                best_d2 = d2
                best_s = s_val

        return best_s

    def path_fcn(self, s):
        """
        Returns the interpolated [x, y] ENU position along the path for parameter s.
        If no path is loaded, defaults to a straight line along x.
        """
        path = self.enu_path if self.enu_path and len(self.enu_path) >= 2 else None
        if not path:
            return [s, 0.0]
        idx = int(s)
        if idx >= len(path) - 1:
            return path[-1]
        p0 = path[idx]
        p1 = path[idx+1]
        t = s - idx
        return [p0[0] + t*(p1[0]-p0[0]), p0[1] + t*(p1[1]-p0[1])]

    def path_fcn_dot(self, s):
        """
        Returns the derivative [dx, dy] of the ENU path at parameter s.
        Used for LOS guidance direction.
        """
        path = self.enu_path if self.enu_path and len(self.enu_path) >= 2 else None
        if not path:
            return [1.0, 0.0]
        idx = int(s)
        if idx >= len(path) - 1:
            idx = len(path) - 2
        p0 = path[idx]
        p1 = path[idx+1]
        return [p1[0]-p0[0], p1[1]-p0[1]]

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

    def blueboat_gps_callback(self, msg):
        """Handle GPS position updates from the ASV."""
        # Log GPS position
        self.get_logger().info(f'ASV GPS Position - Lat/Lon: ({msg.latitude:.6f}, {msg.longitude:.6f}, alt: {msg.altitude:.2f})')
        
        # Store the latest GPS position
        self.blueboat_gps = msg

    def blueboat_odom_callback(self, msg):
        """Handle odometry updates from the ASV."""
        self.blueboat_odom = msg
        
        # Get position in ENU
        pos = msg.pose.pose.position
        
        # Log position information
        self.get_logger().info(f'ASV Position - ENU: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')
        
        # Only proceed with control if we have all necessary data
        if hasattr(self, 'blueboat_gps') and hasattr(self, 'rov_odom'):
            self.control_step()

    def calculate_relative_position(self):
        """Calculate relative position and angle between ASV and ROV."""
        if not hasattr(self, 'blueboat_odom') or not hasattr(self, 'rov_odom'):
            return None, None

        asv_pos = self.blueboat_odom.pose.pose.position
        rov_pos = self.rov_odom.pose.pose.position

        # Calculate relative vector
        dx = rov_pos.x - asv_pos.x
        dy = rov_pos.y - asv_pos.y
        dz = rov_pos.z - asv_pos.z

        # Calculate distance and angle
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        angle = math.atan2(dy, dx)  # Angle in radians from ASV to ROV
        return distance, angle

    def rov_odom_callback(self, msg):
        self.rov_odom = msg
        
        # Get position in ENU
        pos = msg.pose.pose.position
        
        # Log ROV position
        self.get_logger().info(f'ROV Position - ENU: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')

        # Calculate and log relative position only if ASV odometry is available
        if hasattr(self, 'blueboat_odom') and self.blueboat_odom is not None:
            distance, angle = self.calculate_relative_position()
            if distance is not None:
                self.get_logger().info(f'ASV-ROV: Distance = {distance:.2f}m, Angle = {math.degrees(angle):.2f}°')
        else:
            # ASV odometry not received yet; skip relative calculation
            self.get_logger().debug('ASV odometry not yet available - skipping ASV-ROV relative computation')

    def control_step(self):
        """
        Runs the control loop. The controller starts controlling as soon as both USV and ROV odometry are received.
        Path following is always active; if no path is loaded, a straight line is used.
        """
        self.get_logger().info('Starting control step...')
        
        if self.blueboat_odom is None:
            self.get_logger().warn('No ASV odometry data')
            return
        if self.rov_odom is None:
            self.get_logger().warn('No ROV odometry data')
            return
        
        self.get_logger().info('Have both ASV and ROV data, proceeding with control')
        usv_pos = self.blueboat_odom.pose.pose.position
        rov_pos = self.rov_odom.pose.pose.position
        usv_xy = [usv_pos.x, usv_pos.y]
        rov_xy = [rov_pos.x, rov_pos.y]
        dx = rov_xy[0] - usv_xy[0]
        dy = rov_xy[1] - usv_xy[1]
        theta = math.atan2(dy, dx)
        q = self.blueboat_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        usv_heading = math.atan2(siny_cosp, cosy_cosp)

        # Get parameters (see above for descriptions)
        U = self.get_parameter('U').get_parameter_value().double_value
        Delta = self.get_parameter('Delta').get_parameter_value().double_value
        k = self.get_parameter('k').get_parameter_value().double_value
        L = self.get_parameter('L').get_parameter_value().double_value
        epsilon = self.get_parameter('epsilon').get_parameter_value().double_value
        k_v = self.get_parameter('k_v').get_parameter_value().double_value
        k_a = self.get_parameter('k_a').get_parameter_value().double_value
        m_virtual = self.get_parameter('m_virtual').get_parameter_value().double_value

        # Path following
        # Initialize s to closest point on path on first run to avoid large jumps
        max_s_init_jump = self.get_parameter('max_s_init_jump').get_parameter_value().double_value if self.has_parameter('max_s_init_jump') else 1.0
        max_s_dot = self.get_parameter('max_s_dot').get_parameter_value().double_value if self.has_parameter('max_s_dot') else 0.5
        dt = 0.1
        if not self.s_initialized and self.enu_path is not None:
            try:
                desired_s = float(self.find_closest_s(usv_xy))
                ds = desired_s - float(self.s)
                # handle wrapping if s is interpreted circularly
                if ds > (len(self.enu_path) / 2):
                    ds -= len(self.enu_path)
                if ds < -(len(self.enu_path) / 2):
                    ds += len(self.enu_path)
                # clamp initial jump
                if abs(ds) > max_s_init_jump:
                    ds = math.copysign(max_s_init_jump, ds)
                    self.get_logger().info(f'Clamped initial s jump to {ds:.3f}')
                self.s = float(self.s) + ds
                self.get_logger().info(f'Initialized path parameter s to closest point (clamped): {self.s:.3f}')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize s from path: {e}')
            self.s_initialized = True

        los_params = {'U': U, 'Delta': Delta, 'k': k}
        los_output, s_dot = self.los_guidance(usv_xy, self.s, self.path_fcn, self.path_fcn_dot, los_params)
        # Clamp s_dot to avoid sudden parameter jumps
        s_dot_clamped = max(-max_s_dot, min(max_s_dot, s_dot))
        self.s += s_dot_clamped * dt

        x = [theta, 0.0, usv_xy[0], usv_xy[1], los_output[0], los_output[1]]
        # Use persistent zeta (will be adapted over time)
        zeta = self.zeta
        v_ref = los_output
        # Estimate v_ref_dot from previous LOS sample if available
        if self.prev_los is None:
            v_ref_dot = [0.0, 0.0]
        else:
            dt = 0.1
            v_ref_dot = [(v_ref[0] - self.prev_los[0]) / dt, (v_ref[1] - self.prev_los[1]) / dt]

        try:
            pendulum_output, zeta_dot = self.pendulum_adaptive_controller(x, zeta, v_ref, v_ref_dot, L, epsilon, k_v, k_a)
            # integrate parameter estimates
            try:
                self.zeta = (np.array(self.zeta) + np.array(zeta_dot) * 0.1).astype(float)
            except Exception:
                # keep previous zeta if integration fails
                pass

            # pendulum_output is a 2-vector control force; compute forward command as projection onto ASV heading
            u_p = np.array(pendulum_output).reshape(2,)
            desired_heading = math.atan2(u_p[1], u_p[0])
            current_heading = usv_heading
            # current yaw rate from odometry (if available)
            try:
                r = self.blueboat_odom.twist.twist.angular.z
            except Exception:
                r = 0.0

            # compute reference yaw rate from v_ref and v_ref_dot: d/dt atan2(y,x) = (x*y_dot - y*x_dot)/(x^2+y^2)
            denom = v_ref[0]**2 + v_ref[1]**2 + 1e-8
            r_ref = (v_ref[0]*v_ref_dot[1] - v_ref[1]*v_ref_dot[0]) / denom

            # PD heading controller (k_psi, k_r from parameters)
            k_psi = self.get_parameter('k_psi').get_parameter_value().double_value
            k_r = self.get_parameter('k_r').get_parameter_value().double_value
            # wrap heading error
            def wrap_angle(a):
                while a > math.pi:
                    a -= 2*math.pi
                while a < -math.pi:
                    a += 2*math.pi
                return a

            heading_error = wrap_angle(desired_heading - current_heading)
            u_r = -k_psi * heading_error - k_r * (r - r_ref)

        except Exception as e:
            self.get_logger().error(f'Pendulum controller failed: {e}. Using LOS-based fallback command.')
            # fallback to LOS heading if pendulum fails
            desired_heading = math.atan2(los_output[1], los_output[0])
            current_heading = usv_heading
            try:
                r = self.blueboat_odom.twist.twist.angular.z
            except Exception:
                r = 0.0
            # zero reference yaw rate
            r_ref = 0.0
            k_psi = self.get_parameter('k_psi').get_parameter_value().double_value
            k_r = self.get_parameter('k_r').get_parameter_value().double_value
            def wrap_angle(a):
                while a > math.pi:
                    a -= 2*math.pi
                while a < -math.pi:
                    a += 2*math.pi
                return a
            heading_error = wrap_angle(desired_heading - current_heading)
            u_r = -k_psi * heading_error - k_r * (r - r_ref)


        # Convert control force to differential thrust percentage (-100 to 100)
        max_thrust = 100.0  # Maximum thrust percentage
        min_thrust = -100.0  # Minimum thrust percentage (reverse)

        # Ensure pendulum output exists for logging and subsequent mapping
        if 'pendulum_output' not in locals():
            pendulum_output = [0.0, 0.0]
        if 'u_p' not in locals():
            u_p = np.array([0.0, 0.0])

        # Read tunable parameters for thrust mapping
        base_thrust = self.get_parameter('base_thrust').get_parameter_value().double_value
        turn_gain = self.get_parameter('turn_gain').get_parameter_value().double_value
        thrust_scale = self.get_parameter('thrust_scale').get_parameter_value().double_value

        # Implement underactuated mapping from 2D desired force (u_p) to surge F_u and desired heading psi_d
        try:
            # Angle of desired force vector in inertial frame
            phi = math.atan2(float(u_p[1]), float(u_p[0]))
            # If cos(phi - psi) >= 0 use heading = phi and forward = ||u||, else opposite direction
            angle_diff = phi - usv_heading
            # normalize angle_diff to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2*math.pi
            while angle_diff < -math.pi:
                angle_diff += 2*math.pi

            if math.cos(angle_diff) >= 0:
                psi_d = phi
                F_u = float(np.linalg.norm(u_p))
            else:
                psi_d = phi + math.pi
                F_u = -float(np.linalg.norm(u_p))

            # wrap psi_d to [-pi, pi]
            while psi_d > math.pi:
                psi_d -= 2*math.pi
            while psi_d < -math.pi:
                psi_d += 2*math.pi

            # Heading PD controller for yaw moment (map to a moment/command u_r)
            def wrap_angle(a):
                while a > math.pi:
                    a -= 2*math.pi
                while a < -math.pi:
                    a += 2*math.pi
                return a

            heading_error = wrap_angle(psi_d - current_heading)
            # yaw moment command
            u_r = -k_psi * heading_error - k_r * (r - r_ref)

            # Map yaw moment to percent differential using turn_gain and optional sign flip,
            # and surge F_u to forward percent
            turn_sign = self.get_parameter('turn_sign').get_parameter_value().double_value
            turn = turn_gain * float(u_r) * float(turn_sign)
            # Add detailed debug logging to help tune/diagnose heading behaviour
            self.get_logger().info(f'Heading debug: desired={desired_heading:.3f}, current={current_heading:.3f}, error={heading_error:.3f}')
            self.get_logger().info(f'Rates: r={r:.3f}, r_ref={r_ref:.3f}, u_r={u_r:.5f}, turn_gain={turn_gain}, turn_sign={turn_sign}, turn={turn:.3f}')
            forward_percent = thrust_scale * float(F_u)

        except Exception as e:
            # fallback to previous simple mapping if something fails
            self.get_logger().error(f'Underactuated mapping failed: {e}. Falling back to LOS/projection mapping')
            # Compute heading error (desired - current) and wrap
            def wrap_angle(a):
                while a > math.pi:
                    a -= 2*math.pi
                while a < -math.pi:
                    a += 2*math.pi
                return a
            heading_error = wrap_angle(desired_heading - current_heading)
            turn = turn_gain * heading_error
            # ASV forward unit vector (heading)
            fwd_unit = np.array([math.cos(usv_heading), math.sin(usv_heading)])
            try:
                forward_cmd = float(np.dot(u_p, fwd_unit))
            except Exception:
                forward_cmd = 0.0
            forward_percent = thrust_scale * forward_cmd

        # Apply differential thrust: port = base + forward + turn, stbd = base + forward - turn
        port_thrust = base_thrust + forward_percent + turn
        stbd_thrust = base_thrust + forward_percent - turn

        # Clamp values between min and max
        port_thrust = max(min_thrust, min(max_thrust, port_thrust))
        stbd_thrust = max(min_thrust, min(max_thrust, stbd_thrust))

        # Log control information
        self.get_logger().info(f'Pendulum u_p: {pendulum_output}, yaw control u_r: {u_r}')
        self.get_logger().info(f'Control Status:')
        self.get_logger().info(f'  Theta (USV-ROV): {theta:.3f} rad, USV heading: {usv_heading:.3f} rad')
        self.get_logger().info(f'  LOS Guidance v_ref: ({los_output[0]:.2f}, {los_output[1]:.2f}) m/s')
        self.get_logger().info(f'  Path parameter s: {self.s:.2f}')
        # Additional underactuated debug info if available
        try:
            self.get_logger().info(f'  Desired force angle phi: {phi:.3f} rad, psi_d: {psi_d:.3f} rad, Surge F_u: {F_u:.3f}')
        except Exception:
            pass
        self.get_logger().info(f'  Turn command: {turn:.2f}%, Forward %: {forward_percent:.2f}%, Port: {port_thrust:.1f}%, Starboard: {stbd_thrust:.1f}%')

        # Publish thrust commands
        try:
            self.port_thrust_pub.publish(Float64(data=float(port_thrust)))
            self.stbd_thrust_pub.publish(Float64(data=float(stbd_thrust)))
            self.get_logger().info('Thrust published successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to publish thrust: {e}')

        # store previous LOS for derivative estimates
        try:
            self.prev_los = v_ref
        except Exception:
            pass

    def los_guidance(self, p, s, path_fcn, path_fcn_dot, params):
        # LOS guidance law
        U = params['U']
        Delta = params['Delta']
        k = params['k']

        # Path point and derivative
        p_path = path_fcn(s)
        p_path_dot = self.path_fcn_dot(s)  # like ForwardDiff.derivative
        theta_path = math.atan2(p_path_dot[1], p_path_dot[0])
        R_path = [[math.cos(theta_path), -math.sin(theta_path)],
                [math.sin(theta_path),  math.cos(theta_path)]]
        grad_norm = math.sqrt(p_path_dot[0]**2 + p_path_dot[1]**2)

        # Path-following error
        dx = p[0] - p_path[0]
        dy = p[1] - p_path[1]
        e_x = R_path[0][0]*dx + R_path[1][0]*dy
        e_y = R_path[0][1]*dx + R_path[1][1]*dy
        D = math.sqrt(Delta**2 + e_y**2)

        # LOS guidance law
        v_LOS = [
            U / D * (R_path[0][0]*Delta + R_path[0][1]*-e_y),
            U / D * (R_path[1][0]*Delta + R_path[1][1]*-e_y)
        ]

        # Path parameter update
        def saturation(x):
            return x / math.sqrt(1 + x**2)
        s_dot = U / grad_norm * (Delta/D + k * saturation(e_x))

        return v_LOS, s_dot

    def pendulum_adaptive_controller(self, x, zeta, v_ref, v_ref_dot, L, epsilon, k_v, k_a):
        # Simplified adaptive controller for a pendulum system
            import numpy as np
            theta = x[0]
            theta_dot = x[1]
            v0 = np.array(x[4:6])

            Gamma = np.array([np.cos(theta), np.sin(theta)])
            dGamma = np.array([-np.sin(theta), np.cos(theta)])

            # Virtual output velocity
            v = v0 + epsilon * L * theta_dot * dGamma
            v_err = v - v_ref

            v1 = v0 + L * theta_dot * dGamma  # velocity of second mass
            J = np.outer(dGamma, dGamma)      # projection matrix

            # Regressor Y
            term1 = L * theta_dot**2 * Gamma
            term2 = -(theta_dot / (2 * (epsilon - 1))) * ((np.outer(Gamma, dGamma) + np.outer(dGamma, Gamma)) @ v_err)
            term3 = -(J @ (v_ref_dot - k_v * v_err)) / (epsilon - 1)

            Y = np.column_stack([
                term1 + term2 + term3,
                v0,
                L * theta_dot * dGamma,
                np.eye(2),
                J @ v1,
                J,
                k_v * v_err - v_ref_dot
            ])

            # Control force
            u = -Y @ zeta
            # Adaptation law
            zeta_dot = k_a * (Y.T @ v_err)

            return u, zeta_dot
    
    def pendulum_parameter_vector(self, m0, m, c0, c, epsilon, V_c=None):
        """
        Constructs the parameter vector for the adaptive pendulum controller.
        """
        import numpy as np
        if V_c is None:
            V_c = np.zeros(2)

        zeta = np.hstack([
            m * (1 - epsilon) - m0 * epsilon,
            -(c + c0),
            -c,
            (c + c0) * V_c,
            c * (1 + m0 * epsilon / (m * (epsilon - 1))),
            -c * (1 + m0 * epsilon / (m * (epsilon - 1))) * V_c,
            m + m0
        ])
        return zeta

    def heading_control(self, desired_heading, current_heading):
        # Simple proportional heading controller
        k_psi = 1.0
        heading_error = desired_heading - current_heading
        return k_psi * heading_error

    def asv_virtual_mass_controller(self, heading_cmd):
        # Simple virtual mass controller (gain)
        m_virtual = 5.0
        return heading_cmd / m_virtual

    def atan2d(self, vec):
        import math
        return math.atan2(vec[1], vec[0])

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
