#!/usr/bin/env python3
"""Publishes a synthetic flat ground plane under the robot into a voxblox
pointcloud input (once, repeated a few times at startup).

Without this seed the ESDF has a hole under the robot at startup (the lidar
cannot see the ground below the chassis), so pose prediction finds no contact
and planning fails. Real sensor data later corrects the seeded area via normal
TSDF fusion. Same approach as sdf_dp_flipper_planner's ground_plane_seeder.

The cloud is stamped in cloud_frame (robot-attached, above the ground) so
voxblox's TF lookup tilts the assumed plane with the robot.
"""
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class GroundSeedPublisher(Node):
    def __init__(self):
        super().__init__('ground_seed_publisher')
        self.topic = self.declare_parameter('pointcloud_topic', 'pointcloud_1').value
        self.cloud_frame = self.declare_parameter('cloud_frame', 'base_link').value
        self.plane_radius = self.declare_parameter('plane_radius', 1.5).value
        self.point_spacing = self.declare_parameter('point_spacing', 0.025).value
        # Height of cloud_frame origin above the ground. Must stay above voxblox's
        # min_ray_length_m (0.05 in the demo config).
        self.z_offset = self.declare_parameter('z_offset', 0.08).value
        self.repeat_count = self.declare_parameter('repeat_count', 5).value
        repeat_period = self.declare_parameter('repeat_period', 0.3).value
        startup_delay = self.declare_parameter('startup_delay', 3.0).value

        self.pub = self.create_publisher(PointCloud2, self.topic, 5)
        self.cloud = self.build_cloud()
        self.published = 0
        self.timer = self.create_timer(startup_delay, self.start_publishing)
        self.publish_timer = None
        self.repeat_period = repeat_period

    def start_publishing(self):
        self.timer.cancel()
        self.publish_timer = self.create_timer(self.repeat_period, self.publish_once)

    def publish_once(self):
        if self.published >= self.repeat_count:
            self.publish_timer.cancel()
            self.get_logger().info('Ground seeding finished.')
            return
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud)
        self.published += 1
        self.get_logger().info(f'Published ground seed plane ({self.published}/{self.repeat_count}) on {self.topic}')

    def build_cloud(self) -> PointCloud2:
        n = int(2 * self.plane_radius / self.point_spacing) + 1
        data = bytearray()
        count = 0
        for ix in range(n):
            for iy in range(n):
                x = -self.plane_radius + ix * self.point_spacing
                y = -self.plane_radius + iy * self.point_spacing
                data += struct.pack('fff', x, y, -self.z_offset)
                count += 1
        msg = PointCloud2()
        msg.header.frame_id = self.cloud_frame
        msg.height = 1
        msg.width = count
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * count
        msg.data = bytes(data)
        msg.is_dense = True
        return msg


def main():
    rclpy.init()
    node = GroundSeedPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
