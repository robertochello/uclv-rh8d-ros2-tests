<!-- omit in toc -->
# UCLV RH8D ROS 2 Tests


This repository contains some tests, in ROS2 and C++, for controlling the RH8D robotic hand by Seed Robotics. Developed at the Università degli Studi della Campania Luigi Vanvitelli.

<!-- omit in toc -->
## Summary


- [Installation](#installation)
- [Dependencies](#dependencies)
- [Additional Dependencies](#additional-dependencies)
- [Usage](#usage)
    - [Test Nodes](#test-nodes)
        - [Read Motors Positions Test](#read-motors-positions-test)
        - [Move Finger Motor Test](#move-finger-motor-test)
        - [Move Wrist Motor Test](#move-wrist-motor-test)
        - [Read Finger Motor Positions Test](#read-finger-motor-positions-test)
        - [Read Wrist Motor Positions Test](#read-wrist-motor-positions-test)
- [License](#license)
- [Authors](#authors)

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd /path/to/your/ros2/workspace/src
   git clone https://github.com/robertochello/uclv-rh8d-ros2-tests.git
   ```
2. Build the package using `colcon`:
    ```bash
    cd /path/to/your/ros2/workspace
    colcon build --packages-select uclv_rh8d_ros2_tests
    ```
## Dependencies

- [ROS 2](https://index.ros.org/doc/ros2/) - Robot Operating System 2
- `rclcpp` - ROS 2 C++ Client Library


## Additional Dependencies

This project requires additional dependencies from other repositories. Clone the following repositories into your ROS 2 workspace:
1. `uclv-dynamixel-utils` - Library for Dynamixel motor using Dynamixel Protocol 2.0
    ```bash
    cd /path/to/your/ros2/workspace/src
    git clone https://github.com/robertochello/uclv-dynamixel-utils.git
    ```
2. `uclv-seed-robotics-ros` - Interfaces used
    ```bash
    cd /path/to/your/ros2/workspace/src
    git clone https://github.com/robertochello/uclv-seed-robotics-ros.git
    ```
3. `serial` - External library for reading from serial port
    ```bash
    cd /path/to/your/ros2/workspace/src
    git clone https://github.com/robertochello/serial-ros2.git
    ```
    

## Usage

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Authors

- Roberto Chello - [GitHub Profile](https://github.com/robertochello)
