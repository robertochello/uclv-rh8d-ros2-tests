cmake_minimum_required(VERSION 3.8)
project(uclv_rh8d_ros2_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uclv_dynamixel_utils REQUIRED)
find_package(uclv_seed_robotics_ros_interfaces REQUIRED)
find_package(serial REQUIRED)

add_executable(sin_generator_node src/sin_generator_node.cpp)
ament_target_dependencies(
  sin_generator_node
  "rclcpp"
  "geometry_msgs"
)

add_executable(test_moveMotors src/test_moveMotors.cpp)
ament_target_dependencies(
  test_moveMotors
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces"
  "uclv_dynamixel_utils"
)

add_executable(test_readMotorsPositions src/test_readMotorsPositions.cpp)
ament_target_dependencies(
  test_readMotorsPositions
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces"
  "uclv_dynamixel_utils"
)

add_executable(test_moveFingerMotor src/test_moveFingerMotor.cpp)
ament_target_dependencies(
  test_moveFingerMotor
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces"
  "uclv_dynamixel_utils"
  "geometry_msgs"
)

add_executable(test_moveWristMotor src/test_moveWristMotor.cpp)
ament_target_dependencies(
  test_moveWristMotor
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces"
  "uclv_dynamixel_utils"
)

add_executable(test_readFingerMotorPosition src/test_readFingerMotorPosition.cpp)
ament_target_dependencies(
  test_readFingerMotorPosition
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces"
  "uclv_dynamixel_utils"
)

add_executable(test_readWristMotorPosition src/test_readWristMotorPosition.cpp)
ament_target_dependencies(
  test_readWristMotorPosition
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces"
  "uclv_dynamixel_utils"
)

add_executable(test_readFromSensors src/test_readFromSensors.cpp)
ament_target_dependencies(
  test_readFromSensors
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces"
  "uclv_dynamixel_utils"
  "serial"
)


install(TARGETS 
      test_moveMotors
      test_readMotorsPositions
      test_moveFingerMotor
      test_moveWristMotor
      test_readFingerMotorPosition
      test_readWristMotorPosition
      test_readFromSensors
      sin_generator_node
      DESTINATION 
        lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
