#include <iostream>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/position.hpp"

#include "my_library/hand.hpp"




class TestReadWristMotorPosition : public rclcpp::Node
{
public:

    std::shared_ptr<Hand> hand_;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    
    std::string serial_port_ = "/dev/ttyUSB0";
    int baudrate_ = 1000000;
    float protocol_version_ = 2.0;

    rclcpp::Publisher<custom_msg::msg::Position>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    TestReadWristMotorPosition()
    : Node("test_read_finger_motor_position_node")
    {
        hand_ = std::make_shared<Hand>(serial_port_, baudrate_, protocol_version_);
        hand_->setSerialPortLowLatency(serial_port_);
        if(!hand_->initialize()) {
            throw std::runtime_error("Error: Hand not initialized");
        }

        publisher_ = this->create_publisher<custom_msg::msg::Position>("motor_state", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(millisecondsTimer_),
            std::bind(&TestReadWristMotorPosition::publish_state, this)
        );
    }

private:
void publish_state()
    {   
        std::vector<uint8_t> motor_ids_uint8t_vec;
        motor_ids_uint8t_vec.reserve(motor_ids_.size());
        for (size_t i = 0; i < motor_ids_.size(); i++)
        {
            motor_ids_uint8t_vec.push_back(static_cast<uint8_t>(motor_ids_[i]));
        }

        std::vector<uint32_t> motor_pos;
        try{
            motor_pos[0] = hand_->readWristMotorPosition(motor_ids_uint8t_vec[0]);
        }
        catch(...)
        {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ERROR");
            return;
        } 
        

        auto message = custom_msg::msg::Position();
        message.positions.resize(motor_pos.size());
        message.ids = motor_ids_uint8t_vec;
        
        for (size_t i = 0; i < motor_pos.size(); i++)
        {
            message.positions[i] = static_cast<float>(motor_pos[i]);
        }
        
        publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto hand_driver_node = std::make_shared<TestReadWristMotorPosition>();
        rclcpp::spin(hand_driver_node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
