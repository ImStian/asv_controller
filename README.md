# ASV Controller ROS 2 Package

This package provides a ROS 2 node for controlling an Autonomous Surface Vehicle (ASV) towing system using LOS guidance, pendulum adaptive control, heading control, and a virtual mass controller.

## Build Instructions

1. Source your ROS 2 Humble environment:
   
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Build the package using colcon:
   
   ```bash
   colcon build --packages-select asv_controller
   ```

3. Source the package overlay:
   
   ```bash
   source install/setup.bash
   ```

## Run the Node

```bash
ros2 launch asv_controller asv_controller.launch.py
```

## Topics
- Subscribes: `/asv_state` (`std_msgs/msg/Float64`)
- Publishes: `/asv_cmd` (`std_msgs/msg/Float64`)

## License
Apache-2.0
