#!/usr/bin/env python3

import rclpy
import time

from std_srvs.srv import SetBool

from dis_tutorial1.srv import AddTwoInts

mynode = None

def add_two_ints_callback(request, response):
    global mynode
    response.sum = request.a + request.b
    mynode.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    return response

def bool_callback(request, response):
    response.success = True
    response.message = "Service: set_bool set successfully!"

    return response

def main(args=None):
    global mynode

    rclpy.init(args=args)

    mynode = rclpy.create_node("py_simple_server_node")

    addition_server = mynode.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
    bool_server = mynode.create_service(SetBool, 'set_bool', bool_callback)

    mynode.get_logger().info("Server is ready!")
    rclpy.spin(mynode)

    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()