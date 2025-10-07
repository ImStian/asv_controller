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

        # Parameters for LOS
        self.params = {
            "los": 20.0,   # Lookahead distance for LOS (m)
            "U": 5.0,      # Desired speed [m/s]
            "k": 0.3       # Parameter update gain
        }  
        self.path_fcn = lambda s: np.array([s, 0.0])  # path function: straight line along x-axis
        
        # Initialize the state vector x_i
        self.s = 0.0  # Initial path parameter
        self.zeta = np.zeros(7)  # Adjust size based on your parameter vector
        self._prev_control_time = None
        

        # Parameters for the controller
        self.m_virtual = 225.0  # Virtual mass of the ASV (kg) (Keeping close to actual mass)
        self.k_v = 1.0  # Velocity control gain
        self.k_a = 0.5  # Acceleration control gain
        self.L = 3.5  # Length of the tether (m)
        self.epsilon = 0.5  # Small positive constant for adaptive controller
        self.k_psi = 1.0  # Heading control gain
        self.k_r = 1.0  # Yaw rate control gain
        self.declare_parameter('heading_mode', 'LOS') # 'LOS' or 'Path'

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
    def control_law(self):
        "Compute the control forces and state derivaties for the ASV towing controller"

        # Unpack the state
        p0 = self.blueboat_odom.pose.pose.position # position in ENU
        psi = self.blueboat_heading # heading
        v0 = self.blueboat_vel # Velocity of the ASV in the body frame
        r = self.blueboat_angular_vel[2] # Yaw rate (rad/s)
        q = np.array([p0.x, p0.y, psi]) # Full ASV state [x, y, psi]
        q_dot = np.array([v0[0], v0[1], r]) # ASV velocity state [u, v, r]
        _ , theta, theta_dot = self.calculate_relative_position() # Angle and Angular velocity of the pendulum

        # Get current time in seconds
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._prev_control_time is not None:
            dt = now - self._prev_control_time
        else:
            dt = 0.0
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
            self.zeta += zeta_dot * dt  # Update parameter estimates

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
        p_path_dot = jacobian(path_fcn)(s)
        theta_path = math.atan2(p_path_dot[1], p_path_dot[0])
        R_path = np.array([[np.cos(theta_path), -np.sin(theta_path)], 
                           [np.sin(theta_path), np.cos(theta_path)]])
        delta_norm = np.linalg.norm(p_path_dot)

        # Path-following error:
        e = R_path.T @ (p - p_path)
        e_x = e[0]
        e_y = e[1]
        D = np.sqrt(delta**2 + e_y**2)

        # LOS guidance law
        v_LOS = U / D * R_path @ np.array([delta, -e_y])

        # Path parameter update
        s_dot = U / delta_norm * (delta/D * k*self.saturation(e_x))

        return v_LOS, s_dot

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