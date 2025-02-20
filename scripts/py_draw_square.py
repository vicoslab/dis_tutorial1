#!/usr/bin/env python3
"""
Copyright (c) 2009, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty


class State(Enum):
    FORWARD = 0
    STOP_FORWARD = 1
    TURN = 2
    STOP_TURN = 3

class DrawSquare(Node):
    def __init__(self):
        super().__init__('draw_square')

        # Publisher for turtle velocity commands
        self.twist_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Subscriber to turtle pose
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        # Client for the reset service
        self.reset_client = self.create_client(Empty, 'reset')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reset service...')
        req = Empty.Request()
        self.reset_future = self.reset_client.call_async(req)

        # Timer callback at ~16ms intervals
        self.timer = self.create_timer(0.0001, self.timer_callback)

        # State variables
        self.current_pose = Pose()
        self.goal_pose = Pose()
        self.first_pose_set = False
        self.first_goal_set = False
        self.state = State.FORWARD

    def pose_callback(self, msg):
        self.current_pose = msg
        self.first_pose_set = True

    def has_reached_goal(self):
        return (abs(self.current_pose.x - self.goal_pose.x) < 0.1 and
                abs(self.current_pose.y - self.goal_pose.y) < 0.1 and
                abs(self.current_pose.theta - self.goal_pose.theta) < 0.01)

    def has_stopped(self):
        return (abs(self.current_pose.angular_velocity) < 0.0001 and
                abs(self.current_pose.linear_velocity) < 0.0001)

    def print_goal(self):
        self.get_logger().info(f"New goal [{self.goal_pose.x:.2f} {self.goal_pose.y:.2f}, {self.goal_pose.theta:.2f}]")

    def command_turtle(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.twist_pub.publish(twist)

    def stop_forward(self):
        if self.has_stopped():
            self.get_logger().info("Reached goal")
            self.state = State.TURN
            self.goal_pose.x = self.current_pose.x
            self.goal_pose.y = self.current_pose.y
            # Calculate new goal orientation: add 90 degrees (pi/2)
            new_theta = (self.current_pose.theta + math.pi / 2) % (2 * math.pi)
            if new_theta >= math.pi:
                new_theta -= 2 * math.pi
            self.goal_pose.theta = new_theta
            self.print_goal()
        else:
            self.command_turtle(0.0, 0.0)

    def stop_turn(self):
        if self.has_stopped():
            self.get_logger().info("Reached goal")
            self.state = State.FORWARD
            self.goal_pose.x = math.cos(self.current_pose.theta) * 2 + self.current_pose.x
            self.goal_pose.y = math.sin(self.current_pose.theta) * 2 + self.current_pose.y
            self.goal_pose.theta = self.current_pose.theta
            self.print_goal()
        else:
            self.command_turtle(0.0, 0.0)

    def forward(self):
        if self.has_reached_goal():
            self.state = State.STOP_FORWARD
            self.command_turtle(0.0, 0.0)
        else:
            self.command_turtle(1.0, 0.0)

    def turn(self):
        if self.has_reached_goal():
            self.state = State.STOP_TURN
            self.command_turtle(0.0, 0.0)
        else:
            self.command_turtle(0.0, 0.5)

    def timer_callback(self):
        # Ensure the reset service call has completed
        if not self.reset_future.done():
            return

        if not self.first_pose_set:
            return

        # Set the first goal based on the current pose
        if not self.first_goal_set:
            self.first_goal_set = True
            self.state = State.FORWARD
            self.goal_pose.x = math.cos(self.current_pose.theta) * 2 + self.current_pose.x
            self.goal_pose.y = math.sin(self.current_pose.theta) * 2 + self.current_pose.y
            self.goal_pose.theta = self.current_pose.theta
            self.print_goal()

        # State machine for controlling the turtle
        if self.state == State.FORWARD:
            self.forward()
        elif self.state == State.STOP_FORWARD:
            self.stop_forward()
        elif self.state == State.TURN:
            self.turn()
        elif self.state == State.STOP_TURN:
            self.stop_turn()


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
