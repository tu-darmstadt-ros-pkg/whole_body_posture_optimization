#include <gtest/gtest.h>

#include <whole_body_planning_server/util.h>
#include <whole_body_planning_server/path_smoother.h>

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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
