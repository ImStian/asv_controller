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


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
<<<<<<< HEAD

        # Parameters for LOS
        self.params = {
            "los": 20.0,   # Lookahead distance for LOS (m)
            "U": 5.0,      # Desired speed [m/s]
            "k": 0.3       # Parameter update gain
        }  

        
        # Initialize the state vector x_i
        self.s = 0.0  # Initial path parameter
        self.path = None
        self.enu_path = None
        # Declare path_file parameter (empty -> no file)
=======
        # Declare parameters for all controller/system gains
        self.declare_parameter('U', 1.0)
        self.declare_parameter('Delta', 5.0)
        self.declare_parameter('k', 0.5)
        self.declare_parameter('L', 3.5)
        self.declare_parameter('epsilon', 0.5)
        self.declare_parameter('k_v', 1.0)
        self.declare_parameter('k_a', 0.1)
        self.declare_parameter('m_virtual', 1.0)
>>>>>>> parent of 671b9d6 (Implemented Entire Control Code (not tested))
        self.declare_parameter('path_file', '')
        # Reference origin for LLA->ENU conversions (defaults can be overridden via params)
        self.declare_parameter('ref_lat', 0.0)
        self.declare_parameter('ref_lon', 0.0)
        self.declare_parameter('ref_alt', 0.0)
        # Read reference origin values into attributes
        try:
            self.ref_lat = float(self.get_parameter('ref_lat').get_parameter_value().double_value)
            self.ref_lon = float(self.get_parameter('ref_lon').get_parameter_value().double_value)
            self.ref_alt = float(self.get_parameter('ref_alt').get_parameter_value().double_value)
        except Exception:
            # Fallback defaults if parameter read fails
            self.ref_lat = 0.0
            self.ref_lon = 0.0
            self.ref_alt = 0.0

        # Attempt to load a path file if provided
        self.load_path()

        # Parameter estimate vector (matches pendulum regressor columns -> 9 elements)
        self.zeta = np.zeros(9)
        self._prev_control_time = None
        

        # Parameters for the controller
        self.m_virtual = 228.78  # Virtual mass of the ASV (kg) (Keeping close to actual mass)
        self.k_v = 1.0  # Velocity control gain
        self.k_a = 0.05  # Acceleration (adaptation) gain - reduced to improve stability
        self.L = 3.5  # Length of the tether (m)
        self.epsilon = 0.5  # Small positive constant for adaptive controller
        self.k_psi = 1.0  # Heading control gain
        self.k_r = 1.0  # Yaw rate control gain
        self.declare_parameter('heading_mode', 'LOS') # 'LOS' or 'Path'
        self.dt = 1e-6  # Time step for numerical derivatives

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

    ######################## Path FUNCTIONS ################################
    def load_path(self):
        """
        Loads waypoints from a YAML file. The file should look like:
        waypoints:
          - [lat1, lon1]
          - [lat2, lon2]
        Waypoints are latitude/longitude in degrees. They are converted to local ENU coordinates (meters)
        using the reference origin (ref_lat, ref_lon, ref_alt).
        """
        # Safely obtain the path_file parameter (it should be declared in __init__)
        try:
            path_file = self.get_parameter('path_file').get_parameter_value().string_value
        except Exception:
            path_file = ''

        if path_file:
            try:
                with open(path_file, 'r') as f:
                    data = yaml.safe_load(f)
                    self.path = data.get('waypoints', None)  # expects list of [lat, lon]
                if self.path:
                    # Convert to ENU using reference origin (declared above)
                    try:
                        self.enu_path = [self.lla_to_enu(lat, lon, self.ref_lat, self.ref_lon, self.ref_alt) for lat, lon in self.path]
                        self.get_logger().info(f'Loaded path from {path_file} (lat/lon, converted to ENU)')
                    except Exception as e:
                        self.get_logger().error(f'Failed to convert waypoints to ENU: {e}')
                        self.path = None
                        self.enu_path = None
                else:
                    self.get_logger().warn(f'No waypoints found in {path_file}')
                    self.path = None
                    self.enu_path = None
            except Exception as e:
                self.get_logger().error(f'Failed to load path: {e}')
                self.path = None
                self.enu_path = None
        else:
            # No external path provided: leave path as None. The code uses a default analytic path in that case.
            self.path = None
            self.enu_path = None

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
                self.dt = now - self._prev_time
                if self.dt > 1e-6:
                    theta = angle - self._prev_angle
                    # unwrap to [-pi, pi]
                    theta = (theta + math.pi) % (2*math.pi) - math.pi
                    theta_dot = theta / self.dt
                else:
                    theta_dot = 0.0
            else:
                theta_dot = 0.0
            # store for next iteration
            self._prev_angle = angle
            self._prev_time = now

        return distance, angle, theta_dot
    
    ######################## MRAC CONTROLLER ##############################
    def control_law(self):
        "Compute the control forces and state derivaties for the ASV towing controller"

        # Unpack the state
        p0_msg = self.blueboat_odom.pose.pose.position
        p0 = np.array([p0_msg.x, p0_msg.y])  # 2D position
        psi = self.blueboat_heading # heading
        v0 = np.array(self.blueboat_vel[:2])  # 2D velocity [vx, vy]
        r = self.blueboat_angular_vel[2] # Yaw rate (rad/s)
        q = np.array([p0[0], p0[1], psi]) # Full ASV state [x, y, psi]
        q_dot = np.array([v0[0], v0[1], r]) # ASV velocity state [u, v, r]
        _ , theta, theta_dot = self.calculate_relative_position() # Angle and Angular velocity of the pendulum

        # Get current time in seconds
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._prev_control_time is not None:
            self.dt = now - self._prev_control_time
        else:
            self.dt = 0.0
        self._prev_control_time = now

        # Use class member states
        s = self.s  # Path parameter
        zeta = self.zeta  # Parameter estimates vector


        # Headway computation
        Gamma = np.array([np.cos(theta), np.sin(theta)])
        dGamma = np.array([-np.sin(theta), np.cos(theta)])
        p = p0 + self.epsilon * Gamma  # Position of the pendulum mass
        v = v0 + self.epsilon * theta_dot * dGamma  # Velocity of the pendulum mass

        # Compute Reference Velocity (requires LOS)
        v_ref, s_dot = self.line_of_sight(p, s, self.path_fcn, self.params)

        # Approximate acceleration of the pendulum mass (guard against self.dt == 0)
        v_ref_plus, _ = self.line_of_sight(p + self.dt * v, self.s + self.dt * s_dot, self.path_fcn, self.params)
        if self.dt is None or self.dt <= 1e-8:
            v_ref_dot = np.zeros_like(v_ref)
        else:
            v_ref_dot = (v_ref_plus - v_ref) / self.dt

        # Adaption law
        # Build state vector x = [theta, theta_dot, p0_x, p0_y, v0_x, v0_y]
        x_state = np.concatenate(([theta, theta_dot], p0, v0))

        u_p, zeta_dot = self.pendulum_adaptive_controller(
            x_state,
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
            r_ref = (psi_ref_plus - psi_ref) / self.dt

        else:
            self.get_logger().error("Invalid heading_mode parameter. Use 'LOS' or 'Path'.")
            return
        
        # Heading Control Law
        psi_error = ((psi - psi_ref + np.pi) % (2 * np.pi)) - np.pi  # Wrap to [-pi, pi]
        u_r = -self.k_psi * psi_error - self.k_r * (r - r_ref)

        # Control Forces (Requires virtual mass controller)
        u = (u_p, u_r)

        # Safety: if adaptive controller produced an excessively large planar force, fall back
        # to a proportional velocity controller to maintain stability and near-constant speed.
        try:
            u_p_norm = float(np.linalg.norm(u_p))
        except Exception:
            u_p_norm = float('nan')
        if math.isfinite(u_p_norm) and u_p_norm > 1e4:
            # fallback proportional controller (force ~ m_virtual * Kp * velocity_error)
            Kp_fb = 10.0
            vel_err = v_ref - (v0 + self.epsilon * self.L * theta_dot * np.array([-math.sin(theta), math.cos(theta)]))
            u_p_fb = self.m_virtual * Kp_fb * vel_err
            self.get_logger().warning(f'Adaptive u_p too large ({u_p_norm:.3g}), using P-fallback u_p={u_p_fb}')
            u_p = u_p_fb

        # Diagnostics: check magnitudes before passing to virtual mass controller
        try:
            u_p_norm = float(np.linalg.norm(u_p))
        except Exception:
            u_p_norm = float('nan')
        zeta_norm = float(np.linalg.norm(self.zeta)) if hasattr(self, 'zeta') else float('nan')
        if not math.isfinite(u_p_norm) or u_p_norm > 1e3 or zeta_norm > 1e3:
            self.get_logger().warning(f'High control magnitude detected: |u_p|={u_p_norm:.3g}, u_r={u_r:.3g}, |zeta|={zeta_norm:.3g}')

        tau = self.asv_virtual_mass_controller(q, q_dot, psi, self.mdl, u, self.m_virtual)
        T_L, T_R = self.asv_thrust_allocation(tau, self.mdl)


        # Update states with time integration
        if self.dt > 0:
            self.s += s_dot * self.dt  # Update path parameter
            # Keep s in [0,1) since path_function maps s->angle=2*pi*s for the circle
            self.s = self.s % 1.0
            # Clamp adaptation rate to avoid instability
            MAX_ZETA_DOT = 100.0
            try:
                zeta_dot_norm = float(np.linalg.norm(zeta_dot))
            except Exception:
                zeta_dot_norm = float('nan')
            if not math.isfinite(zeta_dot_norm):
                self.get_logger().warning(f'non-finite zeta_dot detected: {zeta_dot}')
                zeta_dot = np.zeros_like(self.zeta)
            elif zeta_dot_norm > MAX_ZETA_DOT:
                zeta_dot = zeta_dot * (MAX_ZETA_DOT / zeta_dot_norm)
                self.get_logger().warning(f'zeta_dot clipped to norm {MAX_ZETA_DOT}')

            self.zeta += zeta_dot * self.dt  # Update parameter estimates

            # Clamp zeta magnitude to avoid runaway parameters
            MAX_ZETA = 20
            try:
                zeta_norm = float(np.linalg.norm(self.zeta))
            except Exception:
                zeta_norm = float('nan')
            if not math.isfinite(zeta_norm) or zeta_norm > MAX_ZETA:
                # scale back to MAX_ZETA
                if math.isfinite(zeta_norm) and zeta_norm > 0:
                    self.zeta = self.zeta * (MAX_ZETA / zeta_norm)
                else:
                    self.zeta = np.zeros_like(self.zeta)
                self.get_logger().warning(f'zeta clipped to norm {MAX_ZETA}')

        # Publish thrust commands
        try:
            self.port_thrust_pub.publish(Float64(data=float(T_L)))
            self.stbd_thrust_pub.publish(Float64(data=float(T_R)))
            self.get_logger().info('Thrust published successfully (T_L: {:.2f}, T_R: {:.2f})'.format(T_L, T_R))
        except Exception as e:
            self.get_logger().error(f'Failed to publish thrust: {e}')
        
        return tau

    def line_of_sight(self, p, s, path_fcn, params):
        # Unpack parameters
        U = params["U"]
        delta = params["los"]
        k = params["k"]

        # Compute the path point and its derivative
        p_path = path_fcn(s)
        if hasattr(self, 'path_fcn_dot') and callable(self.path_fcn_dot):
            p_path_dot = self.path_fcn_dot(s)
        else:
            p_path_dot = jacobian(path_fcn)(s)
        theta_path = math.atan2(p_path_dot[1], p_path_dot[0])
        R_path = np.array([[np.cos(theta_path), -np.sin(theta_path)], 
                           [np.sin(theta_path), np.cos(theta_path)]])
        delta_norm = np.linalg.norm(p_path_dot)
        delta_norm = np.maximum(delta_norm, 1e-6)  # Prevent division by zero

        # Path-following error:
        e = R_path.T @ (p - p_path)
        e_x = e[0]
        e_y = e[1]
        D = np.sqrt(delta**2 + e_y**2)
        D = np.maximum(D, 1e-6)  # Prevent division by zero

        # LOS guidance law
        v_LOS = U / D * R_path @ np.array([delta, -e_y])

        # Path parameter update
        s_dot = U / delta_norm * (delta/D * k*self.saturation(e_x))

        return v_LOS, s_dot

    def pendulum_adaptive_controller(self, x, zeta, v_ref, v_ref_dot, L, epsilon, k_v, k_a):
        """Adaptive controller for the pendulum dynamics."""
        # Validate and unpack states
        x = np.asarray(x).flatten()
        if x.size < 6:
            raise ValueError(f"pendulum_adaptive_controller: expected x with length>=6, got shape {x.shape}, x={x}")

        theta = float(x[0])
        theta_dot = float(x[1])
        p0 = np.asarray(x[2:4], dtype=float).reshape(2,)
        v0 = np.asarray(x[4:6], dtype=float)
        # Defensive fallback: if caller passed malformed v0 (empty or wrong shape), try to use node's stored velocity
        if v0.size != 2:
            try:
                v0 = np.asarray(self.blueboat_vel[:2], dtype=float).reshape(2,)
                self.get_logger().warning(f'pendulum_adaptive_controller: malformed v0 in x, using self.blueboat_vel -> {v0}')
            except Exception:
                v0 = np.zeros(2, dtype=float)
                self.get_logger().warning('pendulum_adaptive_controller: malformed v0 in x and no self.blueboat_vel, using zeros')
        else:
            v0 = v0.reshape(2,)  # Take the velocity components [vx, vy]

        Gamma = np.array([np.cos(theta), np.sin(theta)])
        dGamma = np.array([-np.sin(theta), np.cos(theta)])

        # Virtual output (match original formulation: v = v0 + ε L θ̇ dΓ)
        v = v0 + epsilon * L * theta_dot * dGamma
        v_err = v - v_ref

        v1 = v0 + L * theta_dot * dGamma  # Velocity of the second mass
        J = np.outer(dGamma, dGamma)  # Jacobian matrix

        # Regressor: build a 2 x 9 matrix following the Julia implementation
        # Columns are: [col1, v0, L*theta_dot*dGamma, I(2) (2 cols), J@v1, J (2 cols), final]
        col1 = (L * theta_dot**2 * Gamma
                - theta_dot / (2 * (epsilon - 1)) * ((np.outer(Gamma, dGamma) + np.outer(dGamma, Gamma)) @ v_err)
                - (J @ (v_ref_dot - k_v * v_err)) / (epsilon - 1))

        # Ensure column shapes are (2,) or (2,1); np.column_stack handles mixing 2x1 and 2x2 blocks
        Y = np.column_stack([
            col1,                      # (2,)
            v0,                        # (2,)
            L * theta_dot * dGamma,    # (2,)
            np.eye(2),                 # (2,2) contributes two columns
            (J @ v1),                  # (2,)
            J,                         # (2,2) contributes two columns
            (k_v * v_err - v_ref_dot)  # (2,)
        ])

        # Control force (2-vector)
        u = - Y @ zeta

        # Adaptation law (9-vector)
        zeta_dot = self.update_adaptation(Y, v_err, self.dt)

        return u, zeta_dot
    
    def pendulum_parameter_vector(self, m0, m, c0, c, epsilon, V_c=None):
        """Constructs the parameter vector for the adaptive controller of a pendulum system."""
        if V_c is None:
            V_c = np.zeros(2)

        # Build the parameter vector matching the Julia structure. Some entries are 2-element
        # vectors (those multiplied by V_c), so concatenate and flatten to a 9-element vector.
        zeta_parts = [
            np.array(m * (1 - epsilon) - m0 * epsilon),  # scalar
            np.array(-(c + c0)),                         # scalar
            np.array(-c),                                # scalar
            (c + c0) * np.asarray(V_c),                  # vector (2,)
            np.array(c * (1 + m0 * epsilon / (m * (epsilon - 1)))),  # scalar
            -c * (1 + m0 * epsilon / (m * (epsilon - 1))) * np.asarray(V_c),  # vector (2,)
            np.array(m + m0)                             # scalar
        ]

        # Flatten into a single 1-D parameter vector of length 9
        zeta = np.concatenate([np.atleast_1d(p).reshape(-1) for p in zeta_parts])

        return zeta

    def heading_control(self, desired_heading, current_heading):
        """Deprecated: Heading control is implemented directly in control_law"""
        pass

    def asv_virtual_mass_controller(self, q, q_dot, psi, mdl, u, m_virtual):
        """Virtual mass controller (+ Thrust Allocation) for underactuated ASVs."""
        # Unpack states
        u_p, u_r = u

        # Step1: Converting desired planer force to surge direction
        F_u, psi_d, phi = self.underactuated_transform(u_p, psi)

        # Step2: Desired body-frame force vector
        tau_d = np.array([F_u, 0.0, u_r])

        # Step3: Compute mass and damping terms
        M, b = self.mass_Coriolis(mdl, q, q_dot)

        # Diagnostics: inspect intermediate magnitudes
        try:
            tau_d_norm = float(np.linalg.norm(tau_d))
        except Exception:
            tau_d_norm = float('nan')
        try:
            M_norm = float(np.linalg.norm(M))
        except Exception:
            M_norm = float('nan')
        try:
            b_norm = float(np.linalg.norm(b))
        except Exception:
            b_norm = float('nan')

        if not math.isfinite(tau_d_norm) or tau_d_norm > 1e4:
            self.get_logger().warning(f'Large tau_d detected: |tau_d|={tau_d_norm:.3g}, clamping')
        if not math.isfinite(M_norm) or M_norm > 1e6:
            self.get_logger().warning(f'Large mass matrix norm: |M|={M_norm:.3g}')
        if not math.isfinite(b_norm) or b_norm > 1e6:
            self.get_logger().warning(f'Large damping vector norm: |b|={b_norm:.3g}')

        # Safety clamp tau_d to prevent extreme torques/forces
        MAX_TAU_D = 1e5
        tau_d_clamped = np.clip(tau_d, -MAX_TAU_D, MAX_TAU_D)

        # Step4: Virtual mass dynamic compentation
        # Protect against tiny or non-finite m_virtual
        if not math.isfinite(m_virtual) or abs(m_virtual) < 1e-6:
            self.get_logger().warning(f'invalid m_virtual={m_virtual}, using fallback 1.0')
            m_virtual_safe = 1.0
        else:
            m_virtual_safe = m_virtual

        tau = (M @ tau_d_clamped) / m_virtual_safe + b
        tau[1] = 0.0  # No sway force

        return tau


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

        # Safety clamp thruster commands to a reasonable range to avoid simulation blow-ups
        MAX_THRUST = 100  # units depend on simulator; set conservatively
        if not math.isfinite(T_L) or abs(T_L) > MAX_THRUST:
            self.get_logger().warning(f'Thruster left command clamped: {T_L} -> {max(min(T_L, MAX_THRUST), -MAX_THRUST)}')
            T_L = max(min(T_L, MAX_THRUST), -MAX_THRUST)
        if not math.isfinite(T_R) or abs(T_R) > MAX_THRUST:
            self.get_logger().warning(f'Thruster right command clamped: {T_R} -> {max(min(T_R, MAX_THRUST), -MAX_THRUST)}')
            T_R = max(min(T_R, MAX_THRUST), -MAX_THRUST)

        return T_L, T_R
    
    # --- Helpers ---

    def numerical_derivative(self, f, x, h=1e-6):
        """Simple central difference derivative for scalar x."""
        return (np.array(f(x + h)) - np.array(f(x - h))) / (2 * h)
    
    def saturation(self, x):
        """Implements a smooth saturation function that maps R -> [-1,1]"""
        return x / np.sqrt(1 + x**2)
    
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
    
    def mass_Coriolis(self, mdl, q, q_dot):
        """
        Compute the mass matrix M and damping vector b for the ASV model.
        Equivalent to the EulerLagrangeX._mass_Coriolis! call in Julia. (source: gpt)
        """
        psi = q[2]
        u, v, r = q_dot  # body-frame velocities

        m = mdl["mass"]
        J = mdl["inertia"]
        Xu, Yv, Nr, Yr = mdl["Xu"], mdl["Yv"], mdl["Nr"], mdl["Yr"]
        Dul, Dvl, Drl = mdl["Dul"], mdl["Dvl"], mdl["Drl"]

        # --- Mass matrix (simplified rigid-body + added mass approximation)
        M = np.array([
            [m + Xu, 0, 0],
            [0, m + Yv, m * mdl["xG"]],
            [0, m * mdl["xG"], J + Nr]
        ])

        # --- Coriolis + damping term (simplified)
        C = np.array([
            [0, -m * r, 0],
            [m * r, 0, 0],
            [0, 0, 0]
        ])

        D = np.diag([Dul, Dvl, Drl])

        b = (C + D) @ q_dot

        return M, b

    def update_adaptation(self, Y, v_err, dt):
        """
        σ-modified, normalized adaptation law with projection.
        Keeps parameter drift bounded even under noise or bias.
        """
        # Gains
        k_a = self.k_a              # base adaptation rate (e.g. 1e-3)
        sigma = 0.1                 # leakage gain (try 0.01–0.2)
        zeta_max = 20.0             # absolute bound on each parameter

        # Normalization to avoid step-size blowup
        norm_factor = 1.0 + np.linalg.norm(Y)**2

        # σ-modified adaptation law (with normalization)
        zeta_dot = -k_a * (Y.T @ v_err) / norm_factor - sigma * self.zeta

        # Integrate
        self.zeta += self.dt * zeta_dot

        # Projection (hard bounding)
        self.zeta = np.clip(self.zeta, -zeta_max, zeta_max)

        # Optional diagnostics
        if np.any(np.isnan(self.zeta)):
            self.get_logger().warning("NaN detected in zeta — resetting to zero.")
            self.zeta[:] = 0.0

        return zeta_dot

    

    ####################### Model ################################
    @staticmethod
    def asv_model(dimensions, xG, time_constants, V_current):
        """Create a simple dynamic model of the ASV based on its dimensions and time constants."""
        # ASV dynamic model parameters
        L = dimensions[0] # Length (m)
        W = dimensions[1] # Width (m)
        B = dimensions[2] # Draft (m)
        rho_water = 1025 # Density of water (kg/m^3)

        m = L*W*B*rho_water # Mass (kg)
        J = m*(L**2 + W**2)/12 # Moment of inertia (kg*m^2) Assuming rectangular prism
        Xu = 0.6*m # Surge linear drag
        Yv = 0.4*m # Sway linear drag
        Nr = 0.1*J # Yaw linear drag
        Yr = 0.2*J # Yaw cross-coupling drag

        # Daming coefficients
        Tu = time_constants[0] # Surge time constant (s)
        Tv = time_constants[1] # Sway time constant (s)
        Tr = time_constants[2] # Yaw time constant (s)
        Dul = (m + Xu) / Tu # Surge damping
        Dvl = (m + Yv) / Tv # Sway damping
        Drl = (J + Nr) / Tr # Yaw damping
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
