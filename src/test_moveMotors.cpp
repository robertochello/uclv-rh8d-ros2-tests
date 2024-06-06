#include <iostream>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"

#include "uclv_dynamixel_utils/hand.hpp"

using namespace uclv::dynamixel_utils;




class TestMoveMotors : public rclcpp::Node
{
public:

    std::shared_ptr<Hand> hand_;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    std::string serial_port_ = "/dev/ttyUSB0";
    int baudrate_ = 1000000;
    float protocol_version_ = 2.0;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr subscription_;


    TestMoveMotors()
    : Node("test_move_motors_node")
    {
        hand_ = std::make_shared<Hand>(serial_port_, baudrate_, protocol_version_);
        hand_->setSerialPortLowLatency(serial_port_);
        if(!hand_->initialize()) {
            throw std::runtime_error("Error: Hand not initialized");
        }
        
        subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                "/cmd/motor_position", 1,
                std::bind(&TestMoveMotors::topic_callback, this, std::placeholders::_1)
            );
    }

private:
    void topic_callback(const uclv_seed_robotics_ros_interfaces::msg::MotorPositions::SharedPtr pos) {
        try {
            
            for (size_t i = 0; i < pos->ids.size(); i++)
            {
                if (pos->ids[i] > 30 && pos->ids[i] < 34) {
                    hand_->addWristMotor(pos->ids[i]);
                }
                if (pos->ids[i] > 33 && pos->ids[i] < 39) {
                    hand_->addFingerMotor(pos->ids[i]);
                }
            }
            
            hand_->moveMotors(pos->ids, pos->positions);
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
        auto hand_driver_node = std::make_shared<TestMoveMotors>();
        rclcpp::spin(hand_driver_node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
