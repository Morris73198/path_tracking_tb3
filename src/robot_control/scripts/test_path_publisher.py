#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/path', 10)

        # Create a simple circular path
        self.create_circular_path()

        # Publish path once
        self.publish_path()

        self.get_logger().info(f'Published circular path with {len(self.path)//2} points')

    def create_circular_path(self):
        """Create a circular path"""
        radius = 2.0
        num_points = 50

        path_list = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            path_list.append(x)
            path_list.append(y)

        self.path = path_list

    def create_straight_path(self):
        """Create a straight line path"""
        path_list = []
        for i in range(20):
            x = i * 0.5  # 0.5m intervals
            y = 0.0
            path_list.append(x)
            path_list.append(y)

        self.path = path_list

    def create_square_path(self):
        """Create a square path"""
        path_list = []
        side_length = 3.0
        points_per_side = 10

        # Side 1: x increases, y = 0
        for i in range(points_per_side):
            path_list.append(i * side_length / points_per_side)
            path_list.append(0.0)

        # Side 2: x = side_length, y increases
        for i in range(points_per_side):
            path_list.append(side_length)
            path_list.append(i * side_length / points_per_side)

        # Side 3: x decreases, y = side_length
        for i in range(points_per_side):
            path_list.append(side_length - i * side_length / points_per_side)
            path_list.append(side_length)

        # Side 4: x = 0, y decreases
        for i in range(points_per_side):
            path_list.append(0.0)
            path_list.append(side_length - i * side_length / points_per_side)

        self.path = path_list

    def publish_path(self):
        msg = Float64MultiArray()
        msg.data = self.path
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    path_publisher = PathPublisher()

    # Keep publishing periodically in case the subscriber wasn't ready
    timer_period = 2.0  # seconds
    timer = path_publisher.create_timer(timer_period, path_publisher.publish_path)

    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        pass

    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
