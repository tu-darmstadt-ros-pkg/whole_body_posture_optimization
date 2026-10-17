#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include <whole_body_planning_server/util.h>
#include <whole_body_planning_server/path_smoother.h>
#include <whole_body_planning_server/whole_body_planning_server.h>

using namespace whole_body_planning_server;

namespace {

nav_msgs::msg::Path straightPath(double start_x, double stop_x, double step) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "world";
  for (double x = start_x; x <= stop_x + 1e-9; x += step) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}

TEST(Util, slicePathByDistance) {
  nav_msgs::msg::Path path = straightPath(0.0, 1.0, 0.1);
  // Non-positive distance returns the original path
  EXPECT_EQ(path.poses.size(), slicePath(path, -1.0).poses.size());
  // Slice of 0.5 m keeps ~6 poses (0.0 .. 0.5)
  nav_msgs::msg::Path slice = slicePath(path, 0.55);
  ASSERT_FALSE(slice.poses.empty());
  EXPECT_LT(slice.poses.size(), path.poses.size());
  EXPECT_NEAR(0.5, slice.poses.back().pose.position.x, 1e-6);
}

TEST(Util, slicePathByIndex) {
  nav_msgs::msg::Path path = straightPath(0.0, 1.0, 0.1);
  nav_msgs::msg::Path slice = slicePath(path, 2, 5);
  ASSERT_EQ(3u, slice.poses.size());
  EXPECT_NEAR(0.2, slice.poses.front().pose.position.x, 1e-6);
  EXPECT_NEAR(0.4, slice.poses.back().pose.position.x, 1e-6);
  // Out-of-bounds indices are clamped / rejected
  EXPECT_TRUE(slicePath(path, 20, 25).poses.empty());
  EXPECT_TRUE(slicePath(path, 5, 5).poses.empty());
}

TEST(Util, appendPath) {
  nav_msgs::msg::Path path1 = straightPath(0.0, 0.5, 0.1);
  nav_msgs::msg::Path path2 = straightPath(0.6, 1.0, 0.1);
  size_t expected = path1.poses.size() + path2.poses.size();
  appendPath(path1, path2);
  EXPECT_EQ(expected, path1.poses.size());
}

TEST(Util, euclideanDistance2D) {
  geometry_msgs::msg::Point p1;
  p1.x = 1.0; p1.y = 2.0; p1.z = 100.0;
  geometry_msgs::msg::Point p2;
  p2.x = 4.0; p2.y = 6.0; p2.z = -100.0;
  EXPECT_DOUBLE_EQ(5.0, euclideanDistance2D(p1, p2)); // z is ignored
}

TEST(PathSmoother, smoothedPathKeepsEndpointsAndResolution) {
  // L-shaped path
  nav_msgs::msg::Path path;
  path.header.frame_id = "world";
  for (double x = 0.0; x <= 1.0 + 1e-9; x += 0.25) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  for (double y = 0.25; y <= 1.0 + 1e-9; y += 0.25) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  Pathsmoother3D smoother(true, false);
  smoother.setSmoothedPathDiscretization(0.05);
  nav_msgs::msg::Path smoothed = smoother.smooth(path, false);

  ASSERT_GE(smoothed.poses.size(), 2u);
  // Start and goal positions are preserved
  EXPECT_NEAR(0.0, smoothed.poses.front().pose.position.x, 1e-6);
  EXPECT_NEAR(1.0, smoothed.poses.back().pose.position.x, 0.05);
  EXPECT_NEAR(1.0, smoothed.poses.back().pose.position.y, 0.05);
  // Higher resolution than input
  EXPECT_GT(smoothed.poses.size(), path.poses.size());
}

TEST(PathSmoother, forbiddenReverseKeepsDrivingDirection) {
  // The server always asks for the reverse check; without reverse paths the robot must face along the path
  nav_msgs::msg::Path path = straightPath(0.0, 1.0, 0.25);
  Pathsmoother3D smoother(false, false);
  smoother.setSmoothedPathDiscretization(0.05);
  nav_msgs::msg::Path smoothed = smoother.smooth(path, true);

  ASSERT_GE(smoothed.poses.size(), 3u);
  for (size_t i = 1; i + 1 < smoothed.poses.size(); ++i) {
    const auto& q = smoothed.poses[i].pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    EXPECT_NEAR(0.0, yaw, 1e-3) << "pose " << i;
  }
}

namespace {

Eigen::Isometry3d pose2d(double x, double y, double yaw) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, 0.0);
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

double yawOf(const geometry_msgs::msg::PoseStamped& pose) {
  const auto& q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}

TEST(Util, timedPathKeepsTurnsOnTheSpot) {
  // Drive 0.1 m, turn 90 deg in place in two steps, drive 0.1 m
  std::vector<Eigen::Isometry3d> poses = {pose2d(0, 0, 0), pose2d(0.1, 0, 0), pose2d(0.1, 0, 0.8), pose2d(0.1, 0, 1.57),
                                          pose2d(0.1, 0.1, 1.57)};
  nav_msgs::msg::Path path = timedPath(poses, {0.0, 0.5, 1.0, 1.5, 2.0}, rclcpp::Time(10, 0), "map");
  ASSERT_EQ(5u, path.poses.size());
  EXPECT_NEAR(0.8, yawOf(path.poses[2]), 1e-6);
  EXPECT_NEAR(11.0, rclcpp::Time(path.poses[2].header.stamp).seconds(), 1e-9);
}

TEST(Util, timedPathStampsIncrease) {
  // The second and third pose share a stamp, as at the joint of two trajectory parts
  std::vector<Eigen::Isometry3d> poses = {pose2d(0, 0, 0), pose2d(0.1, 0, 0), pose2d(0.2, 0, 0), pose2d(0.3, 0, 0)};
  nav_msgs::msg::Path path = timedPath(poses, {0.0, 0.5, 0.5, 1.0}, rclcpp::Time(0, 0), "map");
  ASSERT_EQ(3u, path.poses.size());
  for (size_t i = 1; i < path.poses.size(); ++i) {
    EXPECT_LT(rclcpp::Time(path.poses[i - 1].header.stamp), rclcpp::Time(path.poses[i].header.stamp));
  }
}

TEST(Util, timedPathKeepsTheGoal) {
  // A last pose within 1 cm of the previous one replaces it, even when only the start is left
  nav_msgs::msg::Path two = timedPath({pose2d(0, 0, 0), pose2d(0.005, 0, 0)}, {0.0, 0.5}, rclcpp::Time(0, 0), "map");
  ASSERT_EQ(2u, two.poses.size());
  EXPECT_NEAR(0.005, two.poses.back().pose.position.x, 1e-9);
  nav_msgs::msg::Path three = timedPath({pose2d(0, 0, 0), pose2d(0.1, 0, 0), pose2d(0.105, 0, 0)}, {0.0, 0.5, 1.0},
                                        rclcpp::Time(0, 0), "map");
  ASSERT_EQ(2u, three.poses.size());
  EXPECT_NEAR(0.105, three.poses.back().pose.position.x, 1e-9);
}

TEST(Util, dropPosesNearStartSkipsTheOffsetStart) {
  // The robot stands 4 cm beside a reference path sampled every 5 cm
  nav_msgs::msg::Path path = straightPath(1.0, 2.0, 0.05);
  path.poses.front().pose.position.x = 1.04;
  path.poses.front().pose.position.y = 0.04;
  dropPosesNearStart(path, 0.2);
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(1.04, path.poses.front().pose.position.x);
  EXPECT_NEAR(1.25, path.poses[1].pose.position.x, 1e-6);
  EXPECT_NEAR(2.0, path.poses.back().pose.position.x, 1e-6);
}

TEST(Util, dropPosesNearStartKeepsTheGoal) {
  nav_msgs::msg::Path path = straightPath(0.0, 0.1, 0.05);
  dropPosesNearStart(path, 0.2);
  ASSERT_EQ(2u, path.poses.size());
  EXPECT_NEAR(0.1, path.poses.back().pose.position.x, 1e-6);
}

TEST(WholeBodyPlanningServer, shutdownDuringRobotModelLoadThrows) {
  rclcpp::init(0, nullptr);
  // Own namespace so no robot_description publisher on the domain answers.
  auto node = std::make_shared<rclcpp::Node>("planning_server", "planning_server_shutdown_test");
  std::thread stopper([] {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
  });
  EXPECT_ANY_THROW(WholeBodyPlanningServer server(node));
  stopper.join();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
