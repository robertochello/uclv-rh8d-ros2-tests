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
        std::string fx_;
        std::string fy_;
        std::string fz_;

        Sensor(int id, std::string fx, std::string fy, std::string fz)
            : id_(id) ,fx_(fx), fy_(fy), fz_(fz) {}
        Sensor () 
            : id_(0),
            fx_(""),
            fy_(""),
            fz_("") {}

        void update (int id, std::string fx, std::string fy, std::string fz) {
            this->id_;
            this->fx_;
            this->fy_;
            this->fz_;
        }
    private:
};



class TestReadFromSensors : public rclcpp::Node
{
public:

    rclcpp::Time time;

    std::string serial_port_ = "/dev/ttyUSB1";
    int baudrate_ = 1000000;
    uint32_t serial_timeout_ = 1000;
    float protocol_version_ = 2.0;

    std::shared_ptr<serial::Serial> sensor_read_;
    std::string line;

    std::vector<Sensor> sensors;

    std::vector<uint16_t> ids;

    rclcpp::Publisher<custom_msg::msg::Sensors>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

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

        double secs = stamp.seconds();
        double msecs = stamp.nanoseconds() / 1000000;
        // std::stringstream ss;
        // ss << "setepoch," << secs << "," << msecs << "\r\n";
        // std::string command = ss.str();


        setSerialPortLowLatency(serial_port_);
        sensor_read_ = std::make_shared<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(serial_timeout_));

        
        sensor_read_->write("resume\n"); // se fai pausedata da putty

        sensor_read_->write("calibrate\n");
        
        publisher_ = this->create_publisher<custom_msg::msg::Sensors>("sensor_state", 1);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&TestReadFromSensors::publish_state, this)
        );



        int num_sensors = 5;
        sensors.resize(num_sensors);

        
        
        

    }




private:
    void publish_state() {
        
        auto message = custom_msg::msg::Sensors();
        message.header.stamp = rclcpp::Clock{}.now();

        



        line = (sensor_read_->readline());
        if (line == "") {
            // ERROR exception
        } else if (line != "") {
            std::cout << "line: " << line << std::endl;

            std::string token; 
            std::vector<std::string> tokens; 
            char delimiter = ','; 
            std::stringstream ss(line);

            while (getline(ss, token, delimiter)) {
                tokens.push_back(token); 
            } 
            tokens.pop_back(); // tolgo l'ultimo che è vuoto

            message.ids.resize(sensors.size());

            if (tokens[0] == "@") {
                for (size_t i = 1; i < 6; i++)
                {

                    auto fx = tokens[-2+3*i];
                    auto fy = tokens[-1+3*i];
                    auto fz = tokens[0+3*i];
                    message.ids[i] = i+1;
                    message.values[i].x = fx; ///


                } 
            }
        }


        publisher_->publish(message);


        


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
