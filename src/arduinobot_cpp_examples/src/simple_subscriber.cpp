#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "arduinobot_cpp_examples/simple_subscriber.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<my::simple_subscriber>("simple_subscriber",
                                                        "/chatter_cpp", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
}