#!/usr/bin/env python3
"""Interactive start/goal markers for requesting whole-body plans.

Provides two freely movable interactive markers (start_pose, goal_pose).
Moving them does NOT trigger planning. A right-click menu entry
"Plan path (start -> goal)" commits the request.

Two goal modes (parameter 'goal_mode'):
- 'topic' (default): publish the sparse path on 'sparse_goal_path'
  (whole_body_posture_optimization_node densifies and plans, no execution)
- 'action': send a nav2_msgs/FollowPath goal to 'action_name'
  (whole_body_planning_server plans AND executes; the server replaces the
  first pose with the actual robot pose from TF, so the start marker mainly
  determines the initial driving direction)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from interactive_markers import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped, Pose


def yaw_to_quaternion(pose: Pose, yaw: float) -> None:
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)


class PlanGoalMarkerNode(Node):
    def __init__(self):
        super().__init__('plan_goal_marker')
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        start = self.declare_parameter('start_pose', [-1.0, 0.0, 0.5, 0.0]).value  # x y z yaw
        goal = self.declare_parameter('goal_pose', [1.5, 0.0, 0.5, 0.0]).value

        self.goal_mode = self.declare_parameter('goal_mode', 'topic').value
        action_name = self.declare_parameter('action_name', 'whole_body_planner/follow_path').value
        if self.goal_mode == 'action':
            self.action_client = ActionClient(self, FollowPath, action_name)
        else:
            self.path_pub = self.create_publisher(Path, 'sparse_goal_path', 1)

        self.server = InteractiveMarkerServer(self, 'plan_goal_marker')
        self.menu = MenuHandler()
        self.menu.insert('Plan path (start -> goal)', callback=self.plan_callback)

        self.poses = {}
        self.add_marker('start_pose', start, color=(0.1, 0.8, 0.1))
        self.add_marker('goal_pose', goal, color=(0.9, 0.3, 0.1))
        self.server.applyChanges()
        self.get_logger().info("Right-click a marker and select 'Plan path' to request a plan.")

    def add_marker(self, name: str, xyzyaw, color) -> None:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = name
        int_marker.description = name
        int_marker.scale = 0.6
        int_marker.pose.position.x = float(xyzyaw[0])
        int_marker.pose.position.y = float(xyzyaw[1])
        int_marker.pose.position.z = float(xyzyaw[2])
        yaw_to_quaternion(int_marker.pose, float(xyzyaw[3]))
        self.poses[name] = int_marker.pose

        # Visible arrow, also acts as right-click menu target
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = 0.4
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.r, arrow.color.g, arrow.color.b = color
        arrow.color.a = 0.9

        # The arrow is the drag handle for moving in the xy-plane. The right-click
        # menu (MenuHandler) is available on every control of the marker.
        plane = InteractiveMarkerControl()
        plane.orientation.w = math.sqrt(0.5)
        plane.orientation.y = math.sqrt(0.5)
        plane.name = 'move_xy'
        plane.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        plane.always_visible = True
        plane.markers.append(arrow)
        int_marker.controls.append(plane)

        lift = InteractiveMarkerControl()
        lift.orientation.w = math.sqrt(0.5)
        lift.orientation.y = math.sqrt(0.5)
        lift.name = 'move_z'
        lift.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(lift)

        rotate = InteractiveMarkerControl()
        rotate.orientation.w = math.sqrt(0.5)
        rotate.orientation.y = math.sqrt(0.5)
        rotate.name = 'rotate_yaw'
        rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate)

        self.server.insert(int_marker, feedback_callback=self.feedback_callback)
        self.menu.apply(self.server, int_marker.name)

    def feedback_callback(self, feedback) -> None:
        if feedback.marker_name in self.poses:
            self.poses[feedback.marker_name] = feedback.pose

    def plan_callback(self, feedback) -> None:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for name in ('start_pose', 'goal_pose'):
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose = self.poses[name]
            path.poses.append(pose_stamped)
        if self.goal_mode == 'action':
            if not self.action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error('FollowPath action server not available.')
                return
            goal = FollowPath.Goal()
            goal.path = path
            self.action_client.send_goal_async(goal)
        else:
            self.path_pub.publish(path)
        self.get_logger().info(
            f"Requested plan from [{path.poses[0].pose.position.x:.2f}, {path.poses[0].pose.position.y:.2f}] "
            f"to [{path.poses[1].pose.position.x:.2f}, {path.poses[1].pose.position.y:.2f}] ({self.goal_mode})")


def main():
    rclpy.init()
    node = PlanGoalMarkerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
