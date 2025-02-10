#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "arduinobot_cpp_examples/simple_parameter.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<my::simple_parameter>("simple_parameter");
    rclcpp::spin(node);
    rclcpp::shutdown();
}