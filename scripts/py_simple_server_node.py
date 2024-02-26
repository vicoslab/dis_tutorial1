#!/usr/bin/env python3

import rclpy
import time

from dis_tutorial1.srv import AddTwoInts

mynode = None

def add_two_ints_callback(request, response):
    global mynode
    response.sum = request.a + request.b
    mynode.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    return response

def main(args=None):
    global mynode

    rclpy.init(args=args)

    mynode = rclpy.create_node("py_simple_server_node") 
    server = mynode.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)

    mynode.get_logger().info("Server is ready!")
    rclpy.spin(mynode)

    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()