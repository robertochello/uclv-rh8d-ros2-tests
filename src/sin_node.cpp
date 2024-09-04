#include <cmath>  // Include for sine function
#include <chrono> // Include for times
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    float amplitude_ = 1000.0; // Amplitude of the sine wave
    float frequency_ = 1.0;    // Frequency of the sine wave
    float offset_ = 2000.0;    // Offset of the sine wave
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Time t0 = this->now();

    MinimalPublisher()
        : Node("minimal_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("topic", 10);
        timer_ = this->create_wall_timer(
            200ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {   
        rclcpp::Time now = this->now();
    double t = (now - t0).seconds();
        auto message = geometry_msgs::msg::PointStamped();

        float position = amplitude_ * std::sin(2.0 * M_PI * frequency_ * t) + offset_;

        message.header.stamp = this->get_clock()->now();;
        message.point.z = position;

        publisher_->publish(message);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}