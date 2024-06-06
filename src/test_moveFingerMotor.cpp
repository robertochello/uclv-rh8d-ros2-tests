#include <iostream>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"

#include "uclv_dynamixel_utils/hand.hpp"

using namespace uclv::dynamixel_utils;


class TestMoveFingerMotor : public rclcpp::Node
{
public:

    std::shared_ptr<Hand> hand_;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    std::string serial_port_ = "/dev/ttyUSB0";
    int baudrate_ = 1000000;
    float protocol_version_ = 2.0;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr subscription_;


    TestMoveFingerMotor()
    : Node("test_move_finger_motor")
    {
        hand_ = std::make_shared<Hand>(serial_port_, baudrate_, protocol_version_);
        hand_->setSerialPortLowLatency(serial_port_);
        if(!hand_->initialize()) {
            throw std::runtime_error("Error: Hand not initialized");
        }
        
        subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                "/cmd/motor_position", 1,
                std::bind(&TestMoveFingerMotor::topic_callback, this, std::placeholders::_1)
            );
    }

private:
    void topic_callback(const uclv_seed_robotics_ros_interfaces::msg::MotorPositions::SharedPtr pos) {
        try {
            hand_->addFingerMotor(pos->ids[0]); // should not be here, it's only for test moveFingerMotor
            hand_->moveFingerMotor(pos->ids[0], pos->positions[0]);
        }
        catch(...) {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ERROR");
            return;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto hand_driver_node = std::make_shared<TestMoveFingerMotor>();
        rclcpp::spin(hand_driver_node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
