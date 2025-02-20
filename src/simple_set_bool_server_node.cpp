#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <memory>

void set_bool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    response->success = true;
    response->message = "You have managed to set a bool.";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request!");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received data: %d", request->data);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response");
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_bool_server");

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service = node->create_service<std_srvs::srv::SetBool>("set_bool", &set_bool);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to set a bool.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}