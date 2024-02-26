// VICOS LAB - RINS EXERCISE 1

// ROS2 imports
#include "rclcpp/rclcpp.hpp" // The ROS2 CPP library, that should be included in each ROS2 CPP source
#include "std_msgs/msg/string.hpp" // The definition of the message that we are using
//#include "std_msgs/msg/int32.hpp"

// CPP imports
#include <string> // To use CPP string functions and types
#include <iostream>

// We want to work with D, M, Y, h, m, s, ms ....
using namespace std::chrono_literals;

rclcpp::Logger logger1 = rclcpp::get_logger("logger1");

void topic_callback(const std_msgs::msg::String & msg)
    {
      //std::cout << "I heard: " << msg.data.c_str();
      RCLCPP_INFO(logger1, "I heard: '%s'", msg.data.c_str());
    }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize the rclcpp library (exactly once per process)

  auto our_node = rclcpp::Node::make_shared("simple_subscriber_node"); // Create the Node (using ROS2 shortcuts)
  //std::shared_ptr<rclcpp::Node> node = std::shared_ptr<rclcpp::Node>(new rclcpp::Node("string_publisher_node")); // Create the Node using pure CPP notation

  auto subscription = our_node->create_subscription<std_msgs::msg::String>("simple_publisher_node/chat", 10, &topic_callback); //the message type is passed as the class in the <> operator
																						  //topic name is the first argument
																						  //queue size is the second argument
                                                                                          //in this way the topic will be /nodename/topic


  rclcpp::Rate loop_rate(500ms); // create a Rate object that will be used to control the frequency of the loop

  while (rclcpp::ok()) { // repeat this wile ROS2 is active

    RCLCPP_INFO(logger1, "Subscriber: I performed one iteration!");
    rclcpp::spin_some(our_node); // handle subscriptions
    loop_rate.sleep(); // sleep for the appropriate amount of time so that the loop is exectuted at the defined rate
  }

  rclcpp::shutdown();
  return 0;
}