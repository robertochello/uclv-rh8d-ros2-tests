cmake_minimum_required(VERSION 3.8)
project(uclv_rh8d_ros2_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_library REQUIRED)
find_package(custom_msg REQUIRED)

add_executable(test_moveMotors src/test_moveMotors.cpp)
ament_target_dependencies(
  test_moveMotors
  "rclcpp"
  "custom_msg"
  "my_library"
)

add_executable(test_readMotorsPositions src/test_readMotorsPositions.cpp)
ament_target_dependencies(
  test_readMotorsPositions
  "rclcpp"
  "custom_msg"
  "my_library"
)

add_executable(test_moveFingerMotor src/test_moveFingerMotor.cpp)
ament_target_dependencies(
  test_moveFingerMotor
  "rclcpp"
  "custom_msg"
  "my_library"
)

add_executable(test_moveWristMotor src/test_moveWristMotor.cpp)
ament_target_dependencies(
  test_moveWristMotor
  "rclcpp"
  "custom_msg"
  "my_library"
)

add_executable(test_readFingerMotorPosition src/test_readFingerMotorPosition.cpp)
ament_target_dependencies(
  test_readFingerMotorPosition
  "rclcpp"
  "custom_msg"
  "my_library"
)

add_executable(test_readWristMotorPosition src/test_readWristMotorPosition.cpp)
ament_target_dependencies(
  test_readWristMotorPosition
  "rclcpp"
  "custom_msg"
  "my_library"
)


install(TARGETS 
      test_moveMotors
      test_readMotorsPositions
      test_moveFingerMotor
      test_moveWristMotor
      test_readFingerMotorPosition
      test_readWristMotorPosition
      DESTINATION 
        lib/${PROJECT_NAME})

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
