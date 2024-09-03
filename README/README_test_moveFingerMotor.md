# Test Move Finger Motor

This ROS2 (Robot Operating System 2) node is designed to control a motor in a robotic hand using a Dynamixel actuator. The motor can be controlled in three different ways:

1. **By receiving position commands from a ROS2 topic.**
2. **Using hardcoded position values.**
3. **Following a sine wave pattern to create a continuous motion.**

## How It Works

The node initializes the robotic hand and sets up communication with the motor using the Dynamixel SDK. It offers three modes of operation:

### 1. Receiving Commands from a ROS2 Topic

The node subscribes to the topic `/cmd/motor_position` of type `uclv_seed_robotics_ros_interfaces::msg::MotorPositions`. When a message is received on this topic, the node will move the motor to the specified position given in the message.

- **Topic:** `/cmd/motor_position`
- **Message Type:** `uclv_seed_robotics_ros_interfaces::msg::MotorPositions`

### 2. Moving the Motor with Hardcoded Values

If the topic input mode is disabled (`use_topic_` set to `false`), the node can move the motor to a pre-defined hardcoded position (`hardcoded_position_`). This mode allows for quick testing and simple control.

### 3. Moving the Motor Following a Sine Wave Pattern

When the sine wave mode is enabled (`use_sine_wave_` set to `true`), the motor's position will continuously follow a sine wave pattern. This creates a smooth and continuous motion, which is useful for testing or demonstration purposes.

## Parameters

- **`serial_port_`**: The serial port to which the Dynamixel motor is connected (default: `/dev/ttyUSB0`).
- **`baudrate_`**: The baud rate for communication (default: `1000000`).
- **`protocol_version_`**: The protocol version of the Dynamixel SDK (default: `2.0`).
- **`motor_id_`**: The ID of the motor to be controlled (default: `36`).
- **`amplitude_`**: The amplitude of the sine wave for motion control (default: `1000.0`).
- **`frequency_`**: The frequency of the sine wave for motion control (default: `1.0`).

## Usage

To run this node, follow these steps:

1. **Clone the Repository:**

   ```bash
   git clone <repository_url>
   cd <repository_directory>
   ```
2. ***Build the Package:***
3. ***Run the node:***

## Configuration

To change between the modes, modify the following variables in the code:
- `use_topic_`: Set to `true` to enable input from the topic.
- `use_sine_wave_`: Set to `true` to use the sine wave function. If both are `false`, the node will use the hardcoded position.

## Error Handling

- If the hand is not initialized properly, an error will be thrown, and the program will terminate.
- During motor operations, any exceptions caught will be logged with a throttled error message to prevent log flooding.
