#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rclpy.task import Future

from std_msgs.msg import String
from dis_tutorial1.msg import CustomMessage
from dis_tutorial1.srv import AddTwoInts
from std_srvs.srv import SetBool

class CompleteNode(Node):
    def __init__(self):
        super().__init__('py_complete_node')

        timer_period = 0.5 # this is in seconds
        self.count = 1
        self.server_available = False
        self.future = Future()

        self.publisher = self.create_publisher(CustomMessage, "topic1", 10)
        self.subscription = self.create_subscription(String, "topic2", self.topic_callback, 10)
        self.server = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.client = self.create_client(SetBool, "set_bool")

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
    
    def topic_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def timer_callback(self):
        # Process the server response, if it is available
        if self.future.done():
            result = self.future.result()
            self.get_logger().info("SetBool service response:'%s'" % result.message)

        # Send a message
        message = CustomMessage()
        message.content = "Hello! Testing a custom message!"
        message.id = self.count
        self.count += 1
        self.publisher.publish(message)
        self.get_logger().info("Publishing: '%s'" % message.content)

        # Send a service request
        if self.server_available:
            request = SetBool.Request()
            request.data = True
            self.future = self.client.call_async(request)
        else:
            self.get_logger().info("Checking to see if the service is available")
            self.server_available = self.client.service_is_ready()

def main(args=None):
    rclpy.init(args=args)

    mynode = CompleteNode()
    rclpy.spin(mynode)

    rclpy.shutdown()

if __name__ == "__main__":
    main()