#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math
import random

class RealisticLidarPublisher(Node):
    def __init__(self):
        super().__init__('realistic_lidar_publisher')

        self.pc_pub = self.create_publisher(PointCloud2, '/points/raw', 10)
        # 5 Hz to simulate the real lidar emitting rate of velodyne
        self.timer = self.create_timer(0.2, self.publish_pointcloud) 
        
        self.get_logger().info('Realistic LiDAR test publisher started')

       
        self.num_points = 500       
        self.max_range = 3.0        
        self.fov_deg = 180.0        
        self.fov_center_deg = 0.0   

        
        self.hex_center = (0, 0)  
        self.hex_vertices = [
            ( 0.5,   0.0),
            ( 0.25,  0.433),
            (-0.25,  0.433),
            (-0.5,   0.0),
            (-0.25, -0.433),
            ( 0.25, -0.433),
        ]

    def point_in_hex(self, x, y):
        cx, cy = self.hex_center
        xs = [v[0] + cx for v in self.hex_vertices]
        ys = [v[1] + cy for v in self.hex_vertices]
        return min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys)

    def generate_points(self):
        points = []
        cx, cy = self.hex_center
        for _ in range(self.num_points):
            
            angle = random.uniform(0, 2*math.pi)  # 0~360°
            r = random.uniform(0.0, self.max_range * 2)  

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = random.uniform(-0.05, 0.2) 

            
            if random.random() < 0.1: 
                vx, vy = random.choice(self.hex_vertices)
                x = cx + vx * random.uniform(0.1, 0.9)
                y = cy + vy * random.uniform(0.1, 0.9)
                z = random.uniform(0.0, 0.1)

            points.append((x, y, z))
        return points

    def publish_pointcloud(self):
        points = self.generate_points()
        msg = PointCloud2()
        # There is no TF tree designed, I assume Lidar is mount on the robot, from robot's POV
        # when robot is moving forward, the pointcloud is changing and passing from it and it is still
        # Hence the original point is (0,0,0) with frame of map
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = b''.join([struct.pack('fff', *p) for p in points])

        self.pc_pub.publish(msg)
        self.get_logger().info(f'Published {len(points)} points')

def main():
    rclpy.init()
    node = RealisticLidarPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

