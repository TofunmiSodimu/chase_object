import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class MinimalSubscriber(Node):

    def __init__(self):

        # Creates the node.
        super().__init__('object_range')

        lidar_qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth = 10
        )

        self.pos_x = 0
        self.pos_y = 0

        self.angle_pos = Float32()
        self.dist = Float32()

        self.object_pos = self.create_subscription(Point, 'topic', self.get_pixel_pos, 1)
        self.object_pos

        self.lidar_data = self.create_subscription(LaserScan, '/scan', self.get_lidar_data, lidar_qos_profile)

        self.publish_angle_pos = self.create_publisher(Float32, '/angle_pos', 5)
        self.publish_distance = self.create_publisher(Float32MultiArray, '/angle_depth', 5)
    

    def get_lidar_data(self, msg):
        ranges = msg.ranges
        
        size_ranges = len(ranges)
        ratio = size_ranges/360

        print(self.angle_pos.data)

        if self.angle_pos.data == float('inf'):
            self.dist.data = float('inf')
        elif self.angle_pos.data != float('inf'):        
            ratio_index = round(ratio*self.angle_pos.data)
            self.dist.data = np.nanmean(ranges[ratio_index-2:ratio_index+2])
        print(self.dist.data)

        new_message = Float32MultiArray(data = [self.angle_pos.data, self.dist.data])
        self.publish_distance.publish(new_message)


    def get_pixel_pos(self, msg):
        self.pos_x = msg.x
        self.pos_y = msg.y

        print('X position: ', self.pos_x)
        print('Y position: ', self.pos_y)
        print()
        
        if self.pos_x == float('inf') and self.pos_y == float('inf'):
            self.angle_pos.data = float('inf')
        else:
            self.angle_pos.data = (62.2 * self.pos_x/320) - 31.1



def main():
	rclpy.init() #init routine needed for ROS2.
	minimal_subscriber = MinimalSubscriber() #Create class object to be used.
	
	rclpy.spin(minimal_subscriber) # Trigger callback processing.		

	#Clean up and shutdown.
	minimal_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()