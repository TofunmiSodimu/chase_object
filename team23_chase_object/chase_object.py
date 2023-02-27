# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge


class MinimalChaseObject(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Declare subscriber
        self.subscription1 = self.create_subscription(Float32MultiArray,'/angle_depth',self.listener_callback,10)
        self.subscription1  # prevent unused variable warning

        #Declare publisher
        self._vel_publish = self.create_publisher(Twist,'/cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg))
        angle = msg.data[0]
        depth1 = msg.data[1]
        move = Twist()
        move.linear.x = 0.0
        move.angular.z = 0.0
        current = 1.0
        linear_error = depth1 - current
        angular_error = 0.0 - angle

        # Apply controller on angular movement
        if abs(angular_error) > 5.0:

            angular_input = 0.1 * angular_error
            #angular_input = (1.8 * (1 - math.exp(-0.1*(abs(angular_error)**2))))/abs(angular_error)
            
            if angular_input > 0.1:
                angular_input = 0.1
            elif angular_input < -0.1:
                angular_input = -0.1
            move.angular.z = angular_input
            print('angular:', angular_error, angular_input)
        
        # Apply controller on linear movement
        
        if abs(linear_error) > 0.1:

            linear_input = 0.1 * linear_error
            #linear_input = (0.22 * (1 - math.exp(-1*(abs(linear_error)**2))))/abs(linear_error)
            if linear_input > 0.05:
                linear_input = 0.05
            elif linear_input < -0.05:
                linear_input = -0.05

            move.linear.x = linear_input
            print('linear:', linear_error, linear_input)
            
        self._vel_publish.publish(move)



def main(args=None):
    rclpy.init(args=args)

    minimal_chase = MinimalChaseObject()

    rclpy.spin(minimal_chase)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #print("hello?")
    minimal_chase.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()