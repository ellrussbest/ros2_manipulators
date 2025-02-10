#ifndef SIMPLE_PARAMETER_HPP_
#define SIMPLE_PARAMETER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <functional>
#include <vector>

using namespace std::placeholders;

namespace my {
    class simple_parameter : public rclcpp::Node {
    public:
        simple_parameter(std::string&& node_name)
            : Node(node_name)
        {
            declare_parameter("simple_int_param", 28);
            declare_parameter<std::string>("simple_string_param", "hello_ros");
            param_callback_handle_ = add_on_set_parameters_callback(std::bind(&simple_parameter::callback, this, _1));
        }

        rcl_interfaces::msg::SetParametersResult callback(const std::vector<rclcpp::Parameter> & parameters);

    private:
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    };

    rcl_interfaces::msg::SetParametersResult 
    simple_parameter::callback(const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;

        for (const auto & parameter : parameters) {
            if(parameter.get_name() == "simple_int_param" 
               && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed! New value is: " << parameter.as_int());
                result.successful = true;
            }

            if(parameter.get_name() == "simple_string_param" 
               && parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed! New value is: " << parameter.as_string());
                result.successful = true;
            }
        }

        return result;
    }
}  // namespace my
#endif  // SIMPLE_PARAMETER_HPP_