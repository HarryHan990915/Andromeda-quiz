#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

class HexagonPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('hexagon_pointcloud_publisher')

        # 发布点云
        self.pc_pub = self.create_publisher(PointCloud2, '/points/raw', 10)

        # 定时器，每秒发布一次
        self.timer = self.create_timer(1.0, self.publish_pointcloud)
        self.get_logger().info('Publisher started: hexagon + scatter points')

    # ---------- 生成点云 ----------
    def generate_points(self):
        points = []

        # 六边形顶点 (z=0.0)
        hex_vertices = [
            ( 0.5,   0.0),
            ( 0.25,  0.433),
            (-0.25,  0.433),
            (-0.5,   0.0),
            (-0.25, -0.433),
            ( 0.25, -0.433),
        ]
        for vx, vy in hex_vertices:
            points.append((vx, vy, 0.0))

        # 六边形内部点 (z=0.1)
        points += [
            (0.0, 0.0, 0.1),
            (0.1, 0.1, 0.1),
            (-0.1, -0.1, 0.1),
            (0.2, 0.0, 0.1),
        ]

        # 六边形外部前方点 (z=0.1)
        points += [
            (1.0, 0.0, 0.1),
            (1.2, 0.3, 0.1),
            (1.5, -0.2, 0.1),
            (0.8, 0.5, 0.1),
        ]

        # 散落点 (z=0.1)
        points += [
            (-1.0, 0.0, 0.1),
            (0.0, 1.5, 0.1),
            (2.0, -1.0, 0.1),
            (3.0, 0.5, 0.1),
        ]

        return points

    # ---------- 发布点云 ----------
    def publish_pointcloud(self):
        points = self.generate_points()
        msg = PointCloud2()
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
    node = HexagonPointCloudPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
