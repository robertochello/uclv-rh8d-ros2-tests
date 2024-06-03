#include <iostream>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/position.hpp"

#include "my_library/hand.hpp"




class TestMoveWristMotor : public rclcpp::Node
{
public:

    std::shared_ptr<Hand> hand_;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    std::string serial_port_ = "/dev/ttyUSB0";
    int baudrate_ = 1000000;
    float protocol_version_ = 2.0;

    rclcpp::Subscription<custom_msg::msg::Position>::SharedPtr subscription_;


    TestMoveWristMotor()
    : Node("test_move_wrist_motor")
    {
        hand_ = std::make_shared<Hand>(serial_port_, baudrate_, protocol_version_);
        hand_->setSerialPortLowLatency(serial_port_);
        if(!hand_->initialize()) {
            throw std::runtime_error("Error: Hand not initialized");
        }
        
        subscription_ = this->create_subscription<custom_msg::msg::Position>(
                "/cmd/motor_position", 1,
                std::bind(&TestMoveWristMotor::topic_callback, this, std::placeholders::_1)
            );
    }

private:
    void topic_callback(const custom_msg::msg::Position::SharedPtr pos) {
        try {
            hand_->addWristMotor(pos->ids[0]); // should not be here, it's only for test moveWristrMotor
            hand_->moveWristMotor(pos->ids[0], pos->positions[0]);
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
        auto hand_driver_node = std::make_shared<TestMoveWristMotor>();
        rclcpp::spin(hand_driver_node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
