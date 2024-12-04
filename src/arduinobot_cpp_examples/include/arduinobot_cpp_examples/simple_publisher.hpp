#ifndef SIMPLE_PUBLISHER_HPP_
#define SIMPLE_PUBLISHER_HPP_

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace my {
    class simple_publisher : public rclcpp::Node {
    public:
        simple_publisher(std::string&& node_name, std::string&& topic_name,
                        const unsigned int& buffer_size)
            : Node(node_name), counter(0) 
        {
            publisher = create_publisher<std_msgs::msg::String>(topic_name, buffer_size);
            timer = create_wall_timer(1s, std::bind(&simple_publisher::callback, this));
            RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
        }

        void callback();

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
        size_t counter;
    };

    void simple_publisher::callback() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello ROS 2 - counter: " + std::to_string(counter++);
        publisher->publish(msg);
    }
}  // namespace my
#endif  // SIMPLE_PUBLISHER_HPP_