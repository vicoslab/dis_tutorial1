#!/usr/bin/env python3

import rclpy
import time
import random

from dis_tutorial1.srv import AddTwoInts

mynode = None

def main(args=None):
    global mynode

    rclpy.init(args=args)

    mynode = rclpy.create_node("py_simple_client_node") 
    client = mynode.create_client(AddTwoInts, 'add_two_ints')

    request = AddTwoInts.Request()
    
    while rclpy.ok():
        request.a = random.randint(0, 100)
        request.b = random.randint(0, 100)
        
        mynode.get_logger().info("Sending a request!")
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(mynode, future)
        response = future.result()
        mynode.get_logger().info('Result of add_two_ints: for %d + %d = %d'%(request.a, request.b, response.sum))
        
        time.sleep(1)

    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()