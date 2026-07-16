#!/usr/bin/env python3
"""Publishes the robot SRDF on 'robot_description_semantic' (latched), like
srdf_publisher, but injects additional groups that whole-body planning needs.

The whole-body trajectory pipeline requires a planning group that contains the
world virtual joint (e.g. flipper_group_with_world) so that base motion is part
of the trajectory (time parameterization + visualization + execution). Until
this group is added to <robot_type>_moveit_config, this node injects it into
the stock SRDF at runtime.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

DEFAULT_INJECT = """
    <group name="flipper_group_with_world">
        <joint name="world_virtual_joint"/>
        <group name="flipper_group"/>
    </group>
"""


class SrdfWithWorldPublisher(Node):
    def __init__(self):
        super().__init__('srdf_with_world_publisher')
        robot_type = self.declare_parameter('robot_type', 'athena').value
        srdf_file = self.declare_parameter('srdf_file', '').value
        inject_xml = self.declare_parameter('inject_xml', DEFAULT_INJECT).value

        if not srdf_file:
            share = get_package_share_directory(f'{robot_type}_moveit_config')
            srdf_file = f'{share}/config/{robot_type}.srdf'

        self.get_logger().info(f"Loading SRDF file: '{srdf_file}'")
        with open(srdf_file, 'r') as f:
            srdf = f.read()

        if '</robot>' not in srdf:
            raise RuntimeError(f"'{srdf_file}' does not look like an SRDF (no </robot> tag)")
        srdf = srdf.replace('</robot>', inject_xml + '\n</robot>')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, 'robot_description_semantic', qos)
        msg = String()
        msg.data = srdf
        self.pub.publish(msg)
        self.get_logger().info("Published SRDF (with injected world groups) on 'robot_description_semantic'")


def main():
    rclpy.init()
    node = SrdfWithWorldPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
