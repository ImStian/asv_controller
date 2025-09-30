# ASV Towing Controller

ROS2 implementation of an adaptive pendulum-based controller for an autonomous surface vehicle (ASV) towing system. The controller uses line-of-sight (LOS) guidance for path following and an adaptive control scheme for managing the towed payload dynamics.

## Features

- Line-of-sight (LOS) guidance for path following
- Pendulum-based adaptive control for towing dynamics
- ROS2 integration with Gazebo simulation
- Configurable controller parameters
- Support for GPS waypoint following (lat/lon to local ENU conversion)

## Dependencies

This controller is designed to work with the [Marine Robotics Simulation Framework](https://github.com/markusbuchholz/marine-robotics-sim-framework), which provides the simulation environment and ASV/ROV models.

Other dependencies:
- ROS2
- `nav_msgs`
- `std_msgs`
- `numpy`

## Build Instructions

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/your_workspace/src
   git clone https://github.com/ImStian/asv_controller.git
   ```

2. Source your ROS2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the package:
   ```bash
   cd ..
   colcon build --symlink-install --merge-install --packages-select asv_controller
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

Launch the controller node:
```bash
ros2 launch asv_controller asv_controller.launch.py
```

### Parameters

The following parameters can be configured in the launch file or via the ROS2 parameter system:

- `U`: Desired surge speed for LOS guidance (m/s)
- `Delta`: Lookahead distance for LOS guidance (m)
- `k`: Path convergence gain for LOS guidance
- `L`: Pendulum controller parameter (length of 'rod')
- `epsilon`: Pendulum controller adaptation rate
- `k_v`: Velocity feedback gain for pendulum controller
- `k_a`: Adaptation gain for pendulum controller
- `m_virtual`: Virtual mass for heading controller
- `path_file`: Path to YAML file with waypoints
- `ref_lat`: Reference latitude for ENU conversion
- `ref_lon`: Reference longitude for ENU conversion
- `ref_alt`: Reference altitude for ENU conversion

### Waypoint Format

The waypoint file should be in YAML format:
```yaml
waypoints:
  - [lat1, lon1]
  - [lat2, lon2]
  # ...
```

## Topics

### Subscribed Topics
- `/model/blueboat/odometry` (nav_msgs/Odometry): ASV odometry
- `/model/rov/odometry` (nav_msgs/Odometry): ROV/towed payload odometry

### Published Topics
- `/model/blueboat/joint/motor_port_joint/cmd_thrust` (std_msgs/Float64): Port thruster command
- `/model/blueboat/joint/motor_stbd_joint/cmd_thrust` (std_msgs/Float64): Starboard thruster command

## License
Apache-2.0
