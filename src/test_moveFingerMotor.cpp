#include <iostream>
#include <vector>
#include <memory>
#include <cmath>  // Include for sine function
#include <chrono> // Include for time

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"

#include "uclv_dynamixel_utils/hand.hpp"

using namespace uclv::dynamixel_utils;
using namespace std::chrono_literals; // To use time suffixes like 100ms

class TestMoveFingerMotor : public rclcpp::Node
{
public:
    std::shared_ptr<Hand> hand_;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    std::string serial_port_ = "/dev/ttyUSB0";
    int baudrate_ = 1000000;
    float protocol_version_ = 2.0;

    std::vector<int64_t> motor_ids_{36};
    int millisecondsTimer_ = 2;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr subscription_;

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;    // Timer for publisher
    rclcpp::TimerBase::SharedPtr timerSin_; // Timer for sine function

    bool use_topic_ = true;      // Set to true to use topic input
    bool use_sine_wave_ = false; // Set to true to use sine function

    uint8_t motor_id_ = 36; // Motor ID used in both modes (hardcoded - topic)

    // Parameters for sine function
    float amplitude_ = 1000.0; // Amplitude of the sine wave
    float frequency_ = 1.0;    // Frequency of the sine wave
    float offset_ = 2000.0;    // Offset of the sine wave
    float time_ = 0.0;         // Initial time

    // Hardcoded position for the motor
    float hardcoded_position_ = 100.0; // Example of hardcoded position

    TestMoveFingerMotor()
        : Node("test_move_finger_motor")
    {
        hand_ = std::make_shared<Hand>(serial_port_, baudrate_, protocol_version_);
        hand_->setSerialPortLowLatency(serial_port_);
        if (!hand_->initialize())
        {
            throw std::runtime_error("Error: Hand not initialized");
        }

        // Subscription to topic
        subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "/cmd/motor_position", 1,
            std::bind(&TestMoveFingerMotor::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>("motor_state", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(millisecondsTimer_),
            std::bind(&TestMoveFingerMotor::publish_state, this));

        // If not using the topic, choose between sine function or hardcoded values
        if (!use_topic_)
        {
            hand_->addFingerMotor(motor_id_); // Add motor at the start
            if (use_sine_wave_)
            {
                start_sine_wave_timer(); // Use sine function
            }
            else
            {
                move_motor_with_hardcoded_values(); // Use hardcoded values
            }
        }
    }

private:
    void topic_callback(const uclv_seed_robotics_ros_interfaces::msg::MotorPositions::SharedPtr pos)
    {
        if (use_topic_)
        { // Execute only if the flag is set to use the topic
            try
            {
                if (!pos->ids.empty() && !pos->positions.empty())
                {
                    hand_->addFingerMotor(pos->ids[0]);
                    hand_->moveFingerMotor(pos->ids[0], pos->positions[0]);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Received message is empty.");
                }
            }
            catch (...)
            {
                RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error in topic callback");
            }
        }
    }

    void move_motor_with_hardcoded_values()
    {
        try
        {
            // Use the hardcoded position to move the motor
            hand_->moveFingerMotor(motor_id_, hardcoded_position_);
        }
        catch (...)
        {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error in moving with hardcoded values");
        }
    }

    void start_sine_wave_timer()
    {
        timerSin_ = this->create_wall_timer(100ms, std::bind(&TestMoveFingerMotor::sine_wave_callback, this));
    }

    void sine_wave_callback()
    {
        // Calculate the new position using the sine function
        float position = amplitude_ * std::sin(2.0 * M_PI * frequency_ * time_) + offset_;

        // Increment time for the next call
        time_ += 0.1; // Increment time (modify this value to change speed)

        try
        {
            // Use the same motor with the common ID
            hand_->moveFingerMotor(motor_id_, position);
        }
        catch (...)
        {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error in moving with sine function");
        }
    }

    void publish_state()
    {
        rclcpp::Time now = this->get_clock()->now();
        std::vector<uint8_t> motor_ids_uint8t_vec;
        motor_ids_uint8t_vec.reserve(motor_ids_.size());
        for (size_t i = 0; i < motor_ids_.size(); i++)
        {
            motor_ids_uint8t_vec.push_back(static_cast<uint8_t>(motor_ids_[i]));
        }

        std::vector<uint32_t> motor_pos;
        try
        {
            motor_pos = hand_->readMotorsPositions(motor_ids_uint8t_vec);
        }
        catch (...)
        {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ERROR");
            return;
        }

        auto message = uclv_seed_robotics_ros_interfaces::msg::MotorPositions();
        message.header.stamp = now;
        message.positions.resize(motor_pos.size());
        message.ids = motor_ids_uint8t_vec;

        for (size_t i = 0; i < motor_pos.size(); i++)
        {
            message.positions[i] = static_cast<float>(motor_pos[i]);
        }

        publisher_->publish(message);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto hand_driver_node = std::make_shared<TestMoveFingerMotor>();
        rclcpp::spin(hand_driver_node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}