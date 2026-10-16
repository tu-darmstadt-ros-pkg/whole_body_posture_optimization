#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <whole_body_posture_optimization/markers.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

using whole_body_posture_optimization::WholeBodyPostureOptimization;
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

class PublishMarkers : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  std::shared_ptr<WholeBodyPostureOptimization> makeOptimizer(const std::string& name, bool publish_markers)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"robot_description", kUrdf}, {"robot_description_semantic", kSrdf},
                                 {"publish_markers", publish_markers}});
    node_ = std::make_shared<rclcpp::Node>(name, options);
    return std::make_shared<WholeBodyPostureOptimization>(node_, nullptr, nullptr);
  }

  rclcpp::Node::SharedPtr node_;
};

}

TEST_F(PublishMarkers, falseCreatesNoMarkerTopicAndPublishesNothing)
{
  auto optimizer = makeOptimizer("disabled", false);
  EXPECT_TRUE(node_->get_publishers_info_by_topic("/disabled/markers").empty());
  EXPECT_FALSE(optimizer->markersEnabled());
  EXPECT_FALSE(optimizer->markersSubscribed());
  optimizer->publishMarkers(nav_msgs::msg::Path(), {});
  optimizer->publishGoalEnd("no plan", Eigen::Vector3d::Zero());
}

TEST_F(PublishMarkers, trueLatchesTheGoalEndWithoutSubscribers)
{
  auto optimizer = makeOptimizer("enabled", true);
  EXPECT_EQ(1u, node_->get_publishers_info_by_topic("/enabled/markers").size());
  EXPECT_TRUE(optimizer->markersEnabled());
  EXPECT_FALSE(optimizer->markersSubscribed());
  optimizer->publishGoalEnd("", Eigen::Vector3d::Zero());

  MarkerArray received;
  auto subscription = node_->create_subscription<MarkerArray>(
      "/enabled/markers", rclcpp::QoS(1).transient_local(), [&received](const MarkerArray& msg) { received = msg; });
  for (int i = 0; i < 200 && received.markers.empty(); ++i) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(optimizer->markersSubscribed());
  ASSERT_EQ(1u, received.markers.size());
  EXPECT_EQ(visualization_msgs::msg::Marker::DELETEALL, received.markers.front().action);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
