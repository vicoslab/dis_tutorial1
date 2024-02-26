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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize the rclcpp library (exactly once per process)

  auto our_node = rclcpp::Node::make_shared("simple_publisher_node"); // Create the Node (using ROS2 shortcuts)
  //std::shared_ptr<rclcpp::Node> node = std::shared_ptr<rclcpp::Node>(new rclcpp::Node("string_publisher_node")); // Create the Node using pure CPP notation

  auto publisher = our_node->create_publisher<std_msgs::msg::String>("~/chat", 10); //the message type is passed as the class in the <> operator
																						  //topic name is the first argument
																						  //queue size is the second argument
                                                                                          //in this way the topic will be /nodename/topic
  /*
  auto publisher = our_node->create_publisher<std_msgs::msg::String>("chat", 10); // this is how we specify an absolute topic within the namespace
  auto publisher = our_node->create_publisher<std_msgs::msg::String>("/chat", 10); // this is how we specify an absolute topic regardless of namespace
  */

  std_msgs::msg::String message; // Create the object that the publisher will publish
  
  std::string message_same = "Hello world of ROS2 publishers! This is message number: ";
  int message_num = 1;

  rclcpp::Rate loop_rate(1s); // create a Rate object that will be used to control the frequency of the loop

  while (rclcpp::ok()) { // repeat this wile ROS2 is active
    message.data = message_same + std::to_string(message_num); // fill in the message that will be used
    publisher->publish(message); // publish the message
    message_num += 1;

    RCLCPP_INFO(logger1, "Publisher: I performed one iteration!");
    rclcpp::spin_some(our_node); // handle subscriptions
    loop_rate.sleep(); // sleep for the appropriate amount of time so that the loop is exectuted at the defined rate
  }

  rclcpp::shutdown();
  return 0;
}