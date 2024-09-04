#include <cmath>   // Include for sine function
#include <chrono>  // Include for timing
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals; // Allows for easy time suffixes (e.g., 100ms)

class SinGenerator : public rclcpp::Node
{
public:
    SinGenerator()
        : Node("sing_generator"), t0_(this->now()) // Initialize t0_ with the current time
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("sin_topic", 10);

        // Timer setup with a callback frequency of 10Hz (100ms)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MinimalPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "MinimalPublisher node has been started.");
    }

private:

    // Sine wave parameters
    float amplitude_ = 1000.0;  // Amplitude of the sine wave
    float frequency_ = 1;       // Frequency of the sine wave (Hz)
    float offset_ = 2000.0;     // Offset of the sine wave

    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Time t0_; 

    void timer_callback()
    {   
        rclcpp::Time now = this->now();  // Get the current time
        double t = (now - t0_).seconds(); // Calculate elapsed time in seconds

        auto message = geometry_msgs::msg::PointStamped();

        // Calculate the sine wave position
        float position = amplitude_ * std::sin(2.0 * M_PI * frequency_ * t) + offset_;

  
        message.header.stamp = now;
        message.point.z = position; 

        // Publish the message
        publisher_->publish(message);

        RCLCPP_DEBUG(this->get_logger(), "Published position: %f at time: %f seconds", position, t);
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MinimalPublisher>();

    // Ensure the logger is set to DEBUG level to see the debug messages
    rclcpp::Logger logger = rclcpp::get_logger("rclcpp");
    rclcpp::Logger::Level log_level = rclcpp::Logger::Level::Debug;
    rclcpp::set_logger_level(logger.get_name(), log_level);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
