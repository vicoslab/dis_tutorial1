// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// CPP imports
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2 imports
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "dis_tutorial1/msg/custom_message.hpp"
#include "dis_tutorial1/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class CompleteNode : public rclcpp::Node
{
public: CompleteNode() : Node("complete_node"), count_(0)
  {
    publisher_ = this->create_publisher<dis_tutorial1::msg::CustomMessage>("topic1", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic2", 10, std::bind(&CompleteNode::topic_callback, this, _1));
    service_ = this->create_service<dis_tutorial1::srv::AddTwoInts>("add_two_ints", std::bind(&CompleteNode::add, this, _1, _2));
    client_ = this->create_client<std_srvs::srv::SetBool>("set_bool");

    timer_ = this->create_wall_timer(2s, std::bind(&CompleteNode::timer_callback, this));
  }

private:
  
  void timer_callback()
  {
    // Send a custom message using the publisher object
    auto message = dis_tutorial1::msg::CustomMessage();
    message.content = "Hello! Testing a custom message!";
    message.id = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.content.c_str());
    publisher_->publish(message);

    if (server_available_){
      //Send a request to a service, using the clinet object
      //Note: This is just an example, you should wait for the service to become available before sending a request!
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>(); // Create the request to be sent
      request->data = true;
      client_->async_send_request(request, std::bind(&CompleteNode::client_callback, this, _1)); // Send the request async
    }else{
      // Check if the service is available
      RCLCPP_INFO(this->get_logger(), "Checking to see if the service is available");
      server_available_ = client_->service_is_ready();
    }
  }

  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  void add(const std::shared_ptr<dis_tutorial1::srv::AddTwoInts::Request> request, std::shared_ptr<dis_tutorial1::srv::AddTwoInts::Response> response){
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
    RCLCPP_INFO(this->get_logger(), "sending back response: [%ld]", (long int)response->sum);
  }

  void client_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFutureWithRequest future){
    auto request_response_pair = future.get();
    RCLCPP_INFO(this->get_logger(), "SetBool service response:'%s'", request_response_pair.second->message.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<dis_tutorial1::msg::CustomMessage>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Service<dis_tutorial1::srv::AddTwoInts>::SharedPtr service_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  size_t count_;
  bool server_available_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompleteNode>());
  rclcpp::shutdown();
  return 0;
}
