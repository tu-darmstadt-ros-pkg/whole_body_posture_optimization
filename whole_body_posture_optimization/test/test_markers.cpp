#include <gtest/gtest.h>

#include <whole_body_posture_optimization/markers.h>

#include <moveit/robot_model/robot_model.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <set>

using namespace whole_body_posture_optimization;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

const char* kUrdf = R"(<robot name="box">
  <link name="base">
    <collision><geometry><box size="0.6 0.4 0.2"/></geometry></collision>
  </link>
</robot>)";
const char* kSrdf = R"(<robot name="box">
  <virtual_joint name="world_virtual_joint" type="floating" parent_frame="map" child_link="base"/>
</robot>)";

moveit::core::RobotModelPtr makeModel()
{
  auto urdf = urdf::parseURDF(kUrdf);
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, kSrdf);
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

PostureOptimizationResult makeResult(const moveit::core::RobotModelPtr& model, double x, bool success)
{
  PostureOptimizationResult result;
  result.requested_pose = Eigen::Isometry3d(Eigen::Translation3d(x, 0.0, 0.2));
  result.result_pose = Eigen::Isometry3d(Eigen::Translation3d(x, 0.0, 0.1));
  result.result_state = std::make_shared<moveit::core::RobotState>(model);
  result.result_state->setToDefaultValues();
  result.result_state->setJointPositions("world_virtual_joint", result.result_pose);
  result.result_state->update();
  result.success = success;
  if (success) {
    // Clockwise from above, as the force angle stability expects
    result.support_polygon.contact_hull_points = {{x + 0.3, -0.2, 0.0}, {x - 0.3, -0.2, 0.0}, {x - 0.3, 0.2, 0.0}, {x + 0.3, 0.2, 0.0}};
    result.result_com = Eigen::Vector3d(0.0, 0.0, 0.1);
    result.stability_arm_opt = 6.0;
    hector_pose_prediction_interface::ContactPointInformation<double> contact;
    contact.point = Eigen::Vector3d(x, 0.2, 0.0);
    result.contact_information.contact_points = {contact, contact};
  }
  return result;
}

nav_msgs::msg::Path makePath(const std::vector<double>& xs)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x : xs) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

std::vector<const Marker*> inNamespace(const MarkerArray& array, const std::string& ns)
{
  std::vector<const Marker*> markers;
  for (const Marker& marker : array.markers) {
    if (marker.ns == ns && marker.action == Marker::ADD) {
      markers.push_back(&marker);
    }
  }
  return markers;
}

class HorizonMarkers : public ::testing::Test {
protected:
  void SetUp() override
  {
    model_ = makeModel();
    markers_ = horizonMarkers(makePath({0.0, 0.5, 1.0}), {makeResult(model_, 0.5, true), makeResult(model_, 1.0, false)},
                              terrain_, model_->getModelFrame());
  }

  moveit::core::RobotModelPtr model_;
  const sdf_contact_estimation::SurfaceBand terrain_{{{0.0, 0.2, 0.0}, {0.05, 0.2, 0.0}, {0.05, 0.3, 0.01}},
                                                     {{0.5, 0.0, -0.3}}};
  MarkerArray markers_;
};

}

TEST(Markers, failureReasonNamesTheCause) {
  PostureOptimizationResult result;
  EXPECT_FALSE(result.success);
  EXPECT_EQ("no posture", failureReason(result));
  result.failure_reason = "infeasible";
  // A support polygon of the last prediction must not turn an infeasible posture into an unstable one
  result.support_polygon.contact_hull_points = {{1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}};
  EXPECT_EQ("infeasible", failureReason(result));
}

TEST_F(HorizonMarkers, startsWithDeleteAllAndUsesTheModelFrame) {
  ASSERT_FALSE(markers_.markers.empty());
  EXPECT_EQ(Marker::DELETEALL, markers_.markers.front().action);
  for (const Marker& marker : markers_.markers) {
    EXPECT_EQ("map", marker.header.frame_id) << marker.ns;
  }
  std::set<std::pair<std::string, int>> keys;
  for (std::size_t i = 1; i < markers_.markers.size(); ++i) {
    EXPECT_TRUE(keys.insert({markers_.markers[i].ns, markers_.markers[i].id}).second) << markers_.markers[i].ns;
  }
}

TEST_F(HorizonMarkers, oneGhostNamespacePerWaypoint) {
  const auto ghost_0 = inNamespace(markers_, "ghost_0");
  const auto ghost_1 = inNamespace(markers_, "ghost_1");
  ASSERT_EQ(1u, ghost_0.size());
  ASSERT_EQ(1u, ghost_1.size());
  EXPECT_NEAR(0.5, ghost_0.front()->pose.position.x, 1e-9);
  EXPECT_NEAR(1.0, ghost_1.front()->pose.position.x, 1e-9);
  EXPECT_GT(ghost_0.front()->color.g, ghost_0.front()->color.r);
  EXPECT_GT(ghost_1.front()->color.r, ghost_1.front()->color.g);
  EXPECT_LT(ghost_0.front()->color.a, 1.0f);
}

TEST_F(HorizonMarkers, pathsFollowRequestedAndResultPoses) {
  const auto reference = inNamespace(markers_, "reference_path");
  ASSERT_EQ(1u, reference.size());
  EXPECT_EQ(3u, reference.front()->points.size());

  const auto planned = inNamespace(markers_, "planned_path");
  ASSERT_EQ(1u, planned.size());
  ASSERT_EQ(2u, planned.front()->points.size());
  ASSERT_EQ(2u, planned.front()->colors.size());
  EXPECT_NEAR(0.1, planned.front()->points[0].z, 1e-9);
  EXPECT_FLOAT_EQ(0.55f, planned.front()->colors[0].g);
  EXPECT_FLOAT_EQ(0.0f, planned.front()->colors[1].g);
}

TEST_F(HorizonMarkers, supportPolygonContactsAndComOfTheStableWaypoint) {
  const auto polygon = inNamespace(markers_, "support_polygon");
  ASSERT_EQ(1u, polygon.size());
  EXPECT_EQ(Marker::LINE_LIST, polygon.front()->type);
  EXPECT_EQ(8u, polygon.front()->points.size());
  ASSERT_EQ(8u, polygon.front()->colors.size());
  // The COM sits above the polygon centre, so every edge is stable
  for (const auto& color : polygon.front()->colors) {
    EXPECT_GT(color.g, 0.0f);
  }

  const auto contacts = inNamespace(markers_, "contacts");
  ASSERT_EQ(1u, contacts.size());
  EXPECT_EQ(Marker::SPHERE_LIST, contacts.front()->type);
  EXPECT_EQ(2u, contacts.front()->points.size());

  const auto com = inNamespace(markers_, "com");
  ASSERT_EQ(2u, com.size());
  EXPECT_NEAR(0.2, com[0]->pose.position.z, 1e-9);
  EXPECT_NEAR(0.0, com[1]->points[1].z, 1e-9);
}

TEST_F(HorizonMarkers, failedPoseAtTheRequestedPoseOfTheFailedWaypoint) {
  const auto failed = inNamespace(markers_, "failed_pose");
  ASSERT_EQ(1u, failed.size());
  EXPECT_EQ(1, failed.front()->id);
  EXPECT_NEAR(1.0, failed.front()->pose.position.x, 1e-9);
  EXPECT_NEAR(0.2, failed.front()->pose.position.z, 1e-9);
}

TEST_F(HorizonMarkers, terrainSurfaceIsOneTranslucentTriangleList) {
  const auto surface = inNamespace(markers_, "terrain_surface");
  ASSERT_EQ(1u, surface.size());
  EXPECT_EQ(Marker::TRIANGLE_LIST, surface.front()->type);
  ASSERT_EQ(terrain_.triangles.size(), surface.front()->points.size());
  EXPECT_NEAR(0.01, surface.front()->points[2].z, 1e-12);
  EXPECT_LT(surface.front()->color.a, 1.0f);
  const MarkerArray without = horizonMarkers(nav_msgs::msg::Path(), {}, {}, "map");
  EXPECT_TRUE(inNamespace(without, "terrain_surface").empty());
  EXPECT_TRUE(inNamespace(without, "terrain_no_surface").empty());
}

TEST_F(HorizonMarkers, missingSurfaceIsRedPoints) {
  const auto none = inNamespace(markers_, "terrain_no_surface");
  ASSERT_EQ(1u, none.size());
  EXPECT_EQ(Marker::POINTS, none.front()->type);
  ASSERT_EQ(1u, none.front()->points.size());
  EXPECT_NEAR(-0.3, none.front()->points[0].z, 1e-12);
  EXPECT_GT(none.front()->color.r, none.front()->color.b);
}

TEST(HorizonMarkersLong, stridesGhostsAndKeepsTheLastOne) {
  const auto model = makeModel();
  std::vector<PostureOptimizationResult> results;
  for (std::size_t k = 0; k <= 2 * kMaxGhosts; ++k) {
    results.push_back(makeResult(model, 0.1 * static_cast<double>(k), true));
  }
  const MarkerArray markers = horizonMarkers(nav_msgs::msg::Path(), results, {}, "map");
  std::set<std::string> ghosts;
  for (const Marker& marker : markers.markers) {
    if (marker.ns.rfind("ghost_", 0) == 0) {
      ghosts.insert(marker.ns);
    }
  }
  EXPECT_LE(ghosts.size(), kMaxGhosts + 1);
  EXPECT_EQ(1u, ghosts.count("ghost_0"));
  EXPECT_EQ(1u, ghosts.count("ghost_" + std::to_string(2 * kMaxGhosts)));
  EXPECT_EQ(results.size(), inNamespace(markers, "planned_path").front()->points.size());
}

TEST(ClearMarkers, singleDeleteAll) {
  const MarkerArray array = clearMarkers("map");
  ASSERT_EQ(1u, array.markers.size());
  EXPECT_EQ(Marker::DELETEALL, array.markers.front().action);
  EXPECT_EQ("map", array.markers.front().header.frame_id);
}

TEST(StatusMarkers, redSphereAndLabelAboveTheRobot) {
  const auto status = statusMarkers(Eigen::Vector3d(1.0, 2.0, 0.3), "unstable", "map");
  ASSERT_EQ(2u, status.size());
  const Marker& sphere = status[0];
  const Marker& label = status[1];
  EXPECT_EQ("status", sphere.ns);
  EXPECT_EQ("status", label.ns);
  EXPECT_NE(sphere.id, label.id);
  EXPECT_EQ(Marker::SPHERE, sphere.type);
  EXPECT_EQ(Marker::TEXT_VIEW_FACING, label.type);
  EXPECT_EQ("unstable", label.text);
  EXPECT_EQ("map", sphere.header.frame_id);
  EXPECT_NEAR(1.0, sphere.pose.position.x, 1e-9);
  EXPECT_NEAR(2.0, sphere.pose.position.y, 1e-9);
  EXPECT_GT(sphere.pose.position.z, 0.3);
  EXPECT_GT(label.pose.position.z, sphere.pose.position.z);
  EXPECT_GT(sphere.color.r, sphere.color.g);
  EXPECT_GT(sphere.color.a, 0.0f);
}

TEST(GoalEndMarkers, abortKeepsTheStatusOtherEndsClear) {
  const MarkerArray cleared = goalEndMarkers("", Eigen::Vector3d(1.0, 2.0, 0.3), "map");
  ASSERT_EQ(1u, cleared.markers.size());
  EXPECT_EQ(Marker::DELETEALL, cleared.markers.front().action);

  const MarkerArray aborted = goalEndMarkers("no posture", Eigen::Vector3d(1.0, 2.0, 0.3), "map");
  ASSERT_EQ(3u, aborted.markers.size());
  EXPECT_EQ(Marker::DELETEALL, aborted.markers[0].action);
  EXPECT_EQ("map", aborted.markers[0].header.frame_id);
  EXPECT_EQ("status", aborted.markers[1].ns);
  EXPECT_NEAR(1.0, aborted.markers[1].pose.position.x, 1e-9);
  EXPECT_EQ("no posture", aborted.markers[2].text);
}

TEST(FailureReason, unstableWithASupportPolygonNoPostureWithout) {
  const auto model = makeModel();
  PostureOptimizationResult unstable = makeResult(model, 0.0, true);
  unstable.success = false;
  EXPECT_EQ("unstable", failureReason(unstable));
  EXPECT_EQ("no posture", failureReason(makeResult(model, 0.0, false)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
