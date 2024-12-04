#ifndef SIMPLE_SUBSCRIBER_HPP_
#define SIMPLE_SUBSCRIBER_HPP_

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::placeholders;

namespace my {
    class simple_subscriber : public rclcpp::Node {
    public:
        simple_subscriber(std::string&& node_name, std::string&& topic, const unsigned int& buffer_size)
            : Node(node_name) 
        {
            subscriber = create_subscription<std_msgs::msg::String>(topic, buffer_size,
                                                                    std::bind(&simple_subscriber::callback, this, _1));
        }

        void callback(const std_msgs::msg::String& msg);

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
    };

    void simple_subscriber::callback(const std_msgs::msg::String& msg) {
        RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data);
    }

}  // namespace my
#endif  // SIMPLE_SUBSCRIBER_HPP_
