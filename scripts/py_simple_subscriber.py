#!/usr/bin/env python3

import rclpy

from std_msgs.msg import String

mynode = None

def topic_callback(msg):
    global mynode
    mynode.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    global mynode
    rclpy.init(args=args)
    mynode = rclpy.create_node("py_simple_subscriber_node")
    
    subscription = mynode.create_subscription(String, "/chat", topic_callback, 10)

    while rclpy.ok():
        rclpy.spin_once(mynode)

    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()