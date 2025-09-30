#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


import yaml
import math
from rclpy.parameter import Parameter

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        # Declare parameters for all controller/system gains
        self.declare_parameter('U', 1.0)
        self.declare_parameter('Delta', 5.0)
        self.declare_parameter('k', 0.5)
        self.declare_parameter('L', 2.0)
        self.declare_parameter('epsilon', 0.5)
        self.declare_parameter('k_v', 1.0)
        self.declare_parameter('k_a', 0.1)
        self.declare_parameter('m_virtual', 1.0)
        self.declare_parameter('path_file', '')

        # Subscribe to Blueboat and ROV odometry
        self.blueboat_odom_sub = self.create_subscription(
            Odometry,
            '/model/blueboat/odometry',
            self.blueboat_odom_callback,
            10)
        self.rov_odom_sub = self.create_subscription(
            Odometry,
            '/model/rov/odometry',
            self.rov_odom_callback,
            10)
        # Publishers for Blueboat thrust
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
                    self.path = data['waypoints']  # expects list of [lat, lon]
                self.enu_path = [self.lla_to_enu(lat, lon, self.ref_lat, self.ref_lon, self.ref_alt) for lat, lon in self.path]
                self.get_logger().info(f'Loaded path from {path_file} (lat/lon, converted to ENU)')
            except Exception as e:
                self.get_logger().error(f'Failed to load path: {e}')
                self.path = None
                self.enu_path = None
        else:
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

    def blueboat_odom_callback(self, msg):
        self.blueboat_odom = msg
        self.control_step()

    def rov_odom_callback(self, msg):
        self.rov_odom = msg
        # Optionally use ROV state in control_step if needed

    def control_step(self):
        """
        Runs the control loop. The controller starts controlling as soon as both USV and ROV odometry are received.
        Path following is always active; if no path is loaded, a straight line is used.
        """
        if self.blueboat_odom is None or self.rov_odom is None:
            return
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
        los_params = {'U': U, 'Delta': Delta, 'k': k}
        los_output, s_dot = self.los_guidance(usv_xy, self.s, self.path_fcn, self.path_fcn_dot, los_params)
        self.s += s_dot * 0.1  # simple integration, dt=0.1s (tune as needed)

        x = [theta, 0.0, usv_xy[0], usv_xy[1], los_output[0], los_output[1]]
        zeta = [1.0, 1.0]
        v_ref = los_output
        v_ref_dot = [0.0, 0.0]
        pendulum_output, _ = self.pendulum_adaptive_controller(x, zeta, v_ref, v_ref_dot, L, epsilon, k_v, k_a)
        desired_heading = self.atan2d(pendulum_output)
        current_heading = usv_heading
        heading_cmd = self.heading_control(desired_heading, current_heading)
        asv_cmd = heading_cmd / m_virtual
        port_thrust = asv_cmd / 2.0
        stbd_thrust = asv_cmd / 2.0
        self.port_thrust_pub.publish(Float64(data=port_thrust))
        self.stbd_thrust_pub.publish(Float64(data=stbd_thrust))
        self.get_logger().info(f'Theta (USV-ROV): {theta:.3f}, USV heading: {usv_heading:.3f}')
        self.get_logger().info(f'Published port: {port_thrust}, stbd: {stbd_thrust}')

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
        m_virtual = 1.0
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
