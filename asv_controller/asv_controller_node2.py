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

        # Parameters for the controller
        self.m_virtual = 50.0  # Virtual mass of the ASV (kg)
        self.los = 20.0  # Lookahead distance for LOS (m)
        self.path_fcn = lambda s: np.array([s, 0.0])  # path function: straight line along x-axis
        self.k_v = 1.0  # Velocity control gain
        self.k_a = 0.5  # Acceleration control gain
        self.L = 3.5  # Length of the tether (m)
        self.epsilon = 0.5  # Small positive constant for adaptive controller
        self.k_psi = 1.0  # Heading control gain
        self.k_r = 1.0  # Yaw rate control gain
        self.declare_parameter('heading_mode', 'LOS') # 'LOS' or 'Path'

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
        if hasattr(self, 'blueboat_gps') and hasattr(self, 'rov_odom'):
            self.control_step()

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
        q = np.array[p0, psi] # Full ASV state
        q_dot = np.array[v0, r] # ASV velocity state
        _ , theta, theta_dot = self.calculate_relative_position() # Angle and Angular velocity of the pendulum

        s = x_i[0] # Path parameter
        zeta = x_i[1:] # Parameter estimates vector


        # Get current time in seconds
        now = self.get_clock().now().nanoseconds * 1e-9
        if hasattr(self, '_prev_time'):
            delta = now - self._prev_time
        else:
            delta = 0.0
        self._prev_time = now


        # Headway computation
        Gamma = np.array([np.cos(theta), np.sin(theta)])
        dGamma = np.array([-np.sin(theta), np.cos(theta)])
        p = p0 + self.epsilon * Gamma  # Position of the pendulum mass
        v = v0 + self.epsilon * theta_dot * dGamma  # Velocity of the pendulum mass

        # Compute Reference Velocity (requires LOS)
        v_ref, s_dot = self.line_of_sight(p, s, self.path_fcn, self.los)

        # Approximate acceleration of the pendulum mass
        v_ref_plus, _ = self.line_of_sight(p + delta * v, self.s + delta * s_dot, self.path_fcn, self.los)
        v_ref_dot = (v_ref_plus - v_ref) / delta

        # Adaptive Control law
        u_p, zeta_dot = self.pendulum_adaptive_controller(
            np.concatenate(([theta, theta_dot], p0, v0)),
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
            r_ref = (psi_ref_plus - psi_ref) / delta

        else:
            self.get_logger().error("Invalid heading_mode parameter. Use 'LOS' or 'Path'.")
            return
        
        # Heading Control Law
        psi_error = ((psi - psi_ref + np.pi) % (2 * np.pi)) - np.pi  # Wrap to [-pi, pi]
        u_r = -self.k_psi * psi_error - self.k_r * (r - r_ref)

        # Control Forces (Requires virtual mass controller)
        u = np.array([u_p, u_r])
            
        tau = self.asv_virtual_mass_controller(q, q_dot, t, mdl, u, self.m_virtual)
        
        x_i_dot = np.concatenate((s_dot, zeta_dot))
        return tau, x_i_dot






    def line_of_sight(self, p, s, path_fcn, path_fcn_dot, params):
        # LOS guidance law
        return v_LOS, s_dot

    def pendulum_adaptive_controller(self, x, zeta, v_ref, v_ref_dot, L, epsilon, k_v, k_a):
        # Simplified adaptive controller for a pendulum system
            return u, zeta_dot
    
    def pendulum_parameter_vector(self, m0, m, c0, c, epsilon, V_c=None):
        return zeta

    def heading_control(self, desired_heading, current_heading):
        return k_psi * heading_error

    def asv_virtual_mass_controller(self, heading_cmd):
        return heading_cmd / m_virtual

    # --- Helpers (numerical derivatives) ---

    def numerical_derivative(f, x, h=1e-6):
        """Simple central difference derivative for scalar x."""
        return (np.array(f(x + h)) - np.array(f(x - h))) / (2 * h)

    ####################### Model ################################
    def asv_model(self):
        # ASV dynamic model parameters
        mdl = {
            'm': 100.0,  # Mass (kg)
            'I_z': 10.0,  # Yaw moment of inertia (kg*m^2)
            'X_u_dot': -20.0,  # Added mass in surge
            'Y_v_dot': -30.0,  # Added mass in sway
            'N_r_dot': -5.0,   # Added mass in yaw
            'D_u': -50.0,     # Linear drag in surge
            'D_v': -100.0,    # Linear drag in sway
            'D_r': -20.0      # Linear drag in yaw
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
