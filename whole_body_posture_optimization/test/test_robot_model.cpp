#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <thread>

#include <whole_body_posture_optimization/robot_model.h>

using whole_body_posture_optimization::RobotModel;

namespace {

const char* kUrdf = R"(<robot name="box">
  <link name="base">
    <collision><geometry><box size="0.6 0.4 0.2"/></geometry></collision>
  </link>
</robot>)";
const char* kSrdf = R"(<robot name="box"/>)";

}

TEST(RobotModel, shutdownDuringSrdfWaitThrows)
{
  // Own context so the shutdown leaves the other tests' context alone.
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  // Own namespace so no robot_description_semantic publisher on the domain answers.
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({{"robot_description", kUrdf}});
  auto node = std::make_shared<rclcpp::Node>("robot_model", "robot_model_shutdown_test", options);
  std::thread stopper([context] {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    context->shutdown("test");
  });
  // MoveIt's SRDF wait ignores the shutdown and runs its 10 s timeout.
  EXPECT_THROW(RobotModel model(node), std::runtime_error);
  stopper.join();
}

TEST(RobotModel, invalidUrdfThrows)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"robot_description", "<robot"}, {"robot_description_semantic", kSrdf}});
  auto node = std::make_shared<rclcpp::Node>("robot_model", "robot_model_invalid_test", options);
  EXPECT_THROW(RobotModel model(node), std::runtime_error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
