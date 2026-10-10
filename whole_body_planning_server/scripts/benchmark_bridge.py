#!/usr/bin/env python3
"""Executes the planning server's whole-body trajectory on the benchmark robot.

The benchmark robot takes flipper position targets on `joint_commands` and
base velocities on `cmd_vel`. This node stands in for the controllers the
planning server expects on the real robot:

  flipper_trajectory_controller/follow_joint_trajectory (FollowJointTrajectory)
      the flipper part, sampled at the time since the trajectory's start stamp
  controller/follow_path (nav2 FollowPath)
      the base part, tracked with the benchmark base_controller's control law
      and gains, so the base is driven as for joint-only methods

The server replans while driving and resends the grown trajectory with the
same start stamp, so a new goal replaces the running one of its kind.
Commands go out only while the episode is active.
"""
import math
import threading

import numpy as np
import rclpy
from benchmark_orchestrator.base_controller import (GOAL_TOLERANCE, INTEGRAL_LIMIT, K_I, K_TH, K_X,
                                                    K_Y, RATE, TimedPath)
from benchmark_orchestrator.scenario import wrap, yaw
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, Float64MultiArray


class JointTrajectory:
    """Linear interpolation of a trajectory_msgs/JointTrajectory in the given joint order."""

    def __init__(self, msg, joint_names, start):
        missing = set(joint_names) - set(msg.joint_names)
        if missing:
            raise ValueError(f'trajectory lacks joints {sorted(missing)}')
        if not msg.points:
            raise ValueError('trajectory has no points')
        columns = [msg.joint_names.index(name) for name in joint_names]
        self.t = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in msg.points])
        self.q = np.array([[p.positions[c] for c in columns] for p in msg.points])
        self.start = start

    def sample(self, now):
        t = (now - self.start).nanoseconds * 1e-9
        return [float(np.interp(t, self.t, self.q[:, j])) for j in range(self.q.shape[1])], t >= self.t[-1]


class BenchmarkBridge(Node):

    def __init__(self):
        super().__init__('benchmark_bridge')
        self.joint_names = self.declare_parameter(
            'joint_names', ['flipper_fl_joint', 'flipper_fr_joint', 'flipper_bl_joint', 'flipper_br_joint']).value
        group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active = False
        self._pose = None
        self._joint_goal = None
        self._joint_trajectory = None
        self._path_goal = None
        self._path = None
        self._k = 0
        self._integral = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/benchmark/episode_active', self._on_active, latched)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=group)
        self._joint_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ActionServer(self, FollowJointTrajectory, 'flipper_trajectory_controller/follow_joint_trajectory',
                     execute_callback=self._execute_joints, goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        ActionServer(self, FollowPath, 'controller/follow_path',
                     execute_callback=self._execute_path, goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.create_timer(1.0 / RATE, self._step, callback_group=group)

    def _on_active(self, msg):
        with self._lock:
            self._active = msg.data
            if not msg.data:
                self._joint_trajectory = self._path = None

    def _on_odom(self, msg):
        p = msg.pose.pose
        with self._lock:
            self._pose = (p.position.x, p.position.y, yaw(p.orientation))

    def _execute_joints(self, goal_handle):
        msg = goal_handle.request.trajectory
        start = Time.from_msg(msg.header.stamp)
        if start.nanoseconds == 0:
            start = self.get_clock().now()
        result = FollowJointTrajectory.Result()
        try:
            trajectory = JointTrajectory(msg, self.joint_names, start)
        except ValueError as exc:
            self.get_logger().error(f'rejected flipper trajectory: {exc}')
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = str(exc)
            goal_handle.abort()
            return result
        self.get_logger().info(f'flipper trajectory: {len(trajectory.t)} points over {trajectory.t[-1]:.2f} s, '
                               f'starts {(self.get_clock().now() - start).nanoseconds * 1e-9:.2f} s ago')
        with self._lock:
            self._joint_goal, self._joint_trajectory = goal_handle, trajectory
        done = self._wait(goal_handle, lambda: self._joint_goal, lambda: self._joint_trajectory is None,
                          self._clear_joints)
        if done:
            goal_handle.succeed()
        return result

    def _execute_path(self, goal_handle):
        result = FollowPath.Result()
        try:
            path = TimedPath(goal_handle.request.path)
        except ValueError as exc:
            self.get_logger().error(f'rejected base path: {exc}')
            result.error_code = FollowPath.Result.INVALID_PATH
            result.error_msg = str(exc)
            goal_handle.abort()
            return result
        stamps = goal_handle.request.path.poses
        self.get_logger().info(f'base path: {len(path.x)} poses, {path.remaining[0]:.2f} m over '
                               f'{(Time.from_msg(stamps[-1].header.stamp) - Time.from_msg(stamps[0].header.stamp)).nanoseconds * 1e-9:.2f} s, '
                               f'max speed {path.v.max():.2f} m/s, from ({path.x[0]:.2f}, {path.y[0]:.2f}, {math.degrees(path.heading[0]):.0f}) '
                               f'to ({path.x[-1]:.2f}, {path.y[-1]:.2f}, {math.degrees(path.heading[-1]):.0f}), robot {self._pose}')
        with self._lock:
            # A replan sends the grown path with the same start, so tracking continues at the same index
            if self._path is None:
                self._k, self._integral = 0, 0.0
            self._k = min(self._k, len(path.x) - 1)
            self._path_goal, self._path = goal_handle, path
        done = self._wait(goal_handle, lambda: self._path_goal, lambda: self._path is None, self._clear_path)
        if done:
            goal_handle.succeed()
        return result

    def _wait(self, goal_handle, current, finished, clear):
        """Blocks until the goal finishes (True), is replaced or canceled (False)."""
        while rclpy.ok():
            with self._lock:
                if current() is not goal_handle:
                    goal_handle.abort()
                    return False
                if goal_handle.is_cancel_requested:
                    clear()
                    goal_handle.canceled()
                    return False
                if finished():
                    return True
            self.get_clock().sleep_for(Duration(seconds=1.0 / RATE))
        return False

    def _clear_joints(self):
        self._joint_trajectory = None

    def _clear_path(self):
        self._path = None
        if self._active:
            self._cmd_pub.publish(Twist())

    def _step(self):
        now = self.get_clock().now()
        with self._lock:
            if not self._active:
                return
            if self._joint_trajectory is not None:
                positions, finished = self._joint_trajectory.sample(now)
                self._joint_pub.publish(Float64MultiArray(data=positions))
                if finished:
                    self._joint_trajectory = None
            if self._path is not None and self._pose is not None:
                self._cmd_pub.publish(self._track())

    def _track(self):
        """One step of base_controller's Kanayama tracking (see benchmark_orchestrator.base_controller)."""
        path = self._path
        x, y, heading = self._pose
        k = self._k = path.nearest(x, y, heading, self._k)
        if k == len(path.x) - 1 and math.hypot(path.x[k] - x, path.y[k] - y) < GOAL_TOLERANCE:
            self._path = None
            return Twist()
        f_x, f_y = path.foot(k, x, y)
        dx, dy = f_x - x, f_y - y
        e_x = math.cos(heading) * dx + math.sin(heading) * dy
        e_y = -math.sin(heading) * dx + math.cos(heading) * dy
        e_th = float(wrap(path.heading[k] - heading))
        if path.v[k] > 0.0:
            self._integral = min(max(self._integral + e_y / RATE, -INTEGRAL_LIMIT), INTEGRAL_LIMIT)
        cmd = Twist()
        cmd.linear.x = path.v[k] * math.cos(e_th) + K_X * e_x
        cmd.angular.z = path.w[k] + path.v[k] * (K_Y * e_y + K_I * self._integral) + K_TH * math.sin(e_th)
        return cmd


def main():
    rclpy.init()
    node = BenchmarkBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
