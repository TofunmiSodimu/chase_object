#Copyright 2016 Open Source Robotics Foundation, Inc.
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
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import sys
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
import math
#angle = 0.0

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_video_subscriber')
        self.angle1 = Float32()
        self.coordinates_x = 0
        self.subscription = self.create_subscription(Point,'topic',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        # Set up QoS Profiles for passing LIDAR info over WiFi
        lidar_qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth = 10
        )

        #Declare that the minimal_subscriber node is subscribing to the /scan topic.
        self._minimal_subscriber = self.create_subscription(LaserScan,'/scan',self.listener_lidar,lidar_qos_profile)
        self._minimal_subscriber  # prevent unused variable warning

        #Declare publisher for angle and depth
        self.publisher1_ = self.create_publisher(Float32MultiArray,'angle_depth',10)

    def listener_callback(self, msg):
        # Convert horizontal coordinates to horizontal angle
        self.coordinates_x = msg.x
        frame_width = 320
        fov = 62.2
        #global angle
        self.angle1.data = ((self.coordinates_x/frame_width) * fov) - 31.1
        print(self.angle1.data)
   
    def listener_lidar(self,msg):
        ranges = msg.ranges
        ranges_size = len(ranges)
        #degrees1 = (((msg.angle_max - msg.angle_min) * 180) / math.pi)
        ratio = ranges_size/360
        depth1_index = round(ratio* self.angle1.data)
        depth1 = np.nanmean(ranges[depth1_index-5 : depth1_index+5])
        if depth1 is None:
            depth1 = 0.0
        print(self.angle1.data,depth1_index,depth1)
        #print(msg.ranges[depth1_index-5:depth1_index+5])
        #new_angle = self.angle1.data - 31.1
        new_angle = self.angle1.data
        new_message = Float32MultiArray(data = [new_angle, depth1])
        #time.sleep(0.5) # seconds
        self.publisher1_.publish(new_message)
        #self.get_logger().info('Publishing: "%s"' %str(new_message))
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()