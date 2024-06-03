#include <iostream>
#include <vector>
#include <memory>

#include "my_library/colors.hpp"

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/sensors.hpp"

#include "serial/serial.h"



class Sensor {
    public:
        int id_;
        int fx_;
        int fy_;
        int fz_;

        Sensor(int id, int fx, int fy, int fz)
            : id_(id) ,fx_(fx), fy_(fy), fz_(fz) {}
    private:
};


class TestReadFromSensors : public rclcpp::Node
{
public:
    
    std::shared_ptr<serial::Serial> sensor_read_;

    std::string serial_port_ = "/dev/ttyUSB1";
    int baudrate_ = 1000000;
    uint32_t serial_timeout_ = 1000;
    float protocol_version_ = 2.0;

    std::string set_epoch_command;

    rclcpp::Subscription<custom_msg::msg::Sensors>::SharedPtr subscription_;

    void setSerialPortLowLatency(const std::string& serial_port) {
        std::cout << "Setting low latency for " << WARN_COLOR << serial_port << CRESET << std::endl;
        std::string command = "setserial " + serial_port + " low_latency";
        int result = system(command.c_str());
        std::cout << "Setted low latency for " << WARN_COLOR << serial_port << CRESET 
                << " result: " << SUCCESS_COLOR << result << CRESET << std::endl;
    }

    TestReadFromSensors()
    : Node("test_read_from_sensors")
    {
        rclcpp::Time stamp = this->now();
std::stringstream ss;
ss << stamp.seconds(); // << "." << stamp.nanoseconds();
std::cout << ss.str() << std::endl;

        set_epoch_command = "setepoch,"+ss; ///////////////////////////////////
        setSerialPortLowLatency(serial_port_);
        sensor_read_ = std::make_shared<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(serial_timeout_));

        sensor_read_->write("calibrate\r\n");
        // sensor_read_->write(se)
        
        subscription_ = this->create_subscription<custom_msg::msg::Sensors>(
                "sensor_state", 1,
                std::bind(&TestReadFromSensors::topic_callback, this, std::placeholders::_1)
            );

        std::string line = (sensor_read_->readline());
        if (line == "") {
            std::cout << "empty " << std::endl;
        } else if (line != "") {
            std::cout << "line: " << line << std::endl;
        }
    }

private:
    void topic_callback(const custom_msg::msg::Sensors::SharedPtr sen) {

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    
    try {
        auto hand_driver_node = std::make_shared<TestReadFromSensors>();
        rclcpp::spin(hand_driver_node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
