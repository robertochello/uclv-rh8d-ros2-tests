# UCLV Robot Hand Controller ROS 2


This repository contains some tests, in ROS2 and C++, for controlling the RH8D robotic hand from Seed Robotics. Developed at the Università degli Studi della Campania Luigi Vanvitelli.

## Dependencies

- [ROS 2](https://index.ros.org/doc/ros2/) - Robot Operating System 2
- `rclcpp` - ROS 2 C++ Client Library
- `custom_msg` - Custom ROS 2 Message package
- `my_library` - External library for robotic hand control

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd /path/to/your/ros2/workspace/src
   git clone https://github.com/robertochello/uclv-robot-hand-controller-ros2.git
   ```
2. Build the package using `colcon`:
    ```bash
    cd /path/to/your/ros2/workspace
    colcon build --packages-select uclv_robot_hand_controller
    ```
## Additional Dependencies

This project requires additional dependencies from other repositories. Clone the following repositories into your ROS 2 workspace:

1. Custom Message Definitions:
   ```bash
    cd /path/to/your/ros2/workspace/src
    git clone https://github.com/robertochello/custom_msg.git
    ```
2. Custom Library:
    ```bash
    cd /path/to/your/ros2/workspace/src
    git clone https://github.com/robertochello/my_library.git
    ```

## Usage

### Test Nodes

#### Move Motors Test

This test node (`test_moveMotors`) is designed to test the functionality of moving motors in the robotic hand.

```bash
    ros2 run uclv_robot_hand_controller test_moveMotors
```

#### Read Motors Positions Test

This test node (`test_readMotorsPositions`) is designed to test reading the positions of motors in the robotic hand.
```bash
    ros2 run uclv_robot_hand_controller test_readMotorsPositions
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Authors

- Roberto Chello - [GitHub Profile](https://github.com/robertochello)
