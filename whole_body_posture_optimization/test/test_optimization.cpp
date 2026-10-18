#include <gtest/gtest.h>

#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost.h>
#include <whole_body_posture_optimization/flipper/flipper_optimization.h>
#include <whole_body_posture_optimization/robot_model.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <whole_body_posture_optimization/util/iterative_spline_parameterization.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <cmath>
#include <limits>
#include <map>

using namespace whole_body_posture_optimization;

namespace {

// One revolute joint without an acceleration limit: the parameterization uses its default of 1 rad/s^2.
const char* kUrdf = R"(<robot name="arm">
  <link name="base"/>
  <link name="tip"/>
  <joint name="j" type="revolute">
    <parent link="base"/><child link="tip"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="1.0" effort="1"/>
  </joint>
</robot>)";
const char* kSrdf = R"(<robot name="arm"><group name="g"><joint name="j"/></group></robot>)";

moveit::core::RobotModelPtr makeModel()
{
  auto urdf = urdf::parseURDF(kUrdf);
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, kSrdf);
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

robot_trajectory::RobotTrajectory rampWithStartAcceleration(const moveit::core::RobotModelPtr& model, double acceleration)
{
  robot_trajectory::RobotTrajectory trajectory(model, "g");
  for (int i = 0; i < 6; ++i) {
    moveit::core::RobotState state(model);
    state.setVariablePosition("j", 0.1 * i);
    if (i == 0) {
      state.setVariableVelocity("j", 0.0);
      state.setVariableAcceleration("j", acceleration);
    }
    trajectory.addSuffixWayPoint(state, 0.0);
  }
  return trajectory;
}

}

TEST(IterativeSplineParameterization, startStateRoundingOverLimitIsAccepted) {
  // A continued trajectory starts with the previous spline's state, which can overshoot by rounding
  auto model = makeModel();
  auto trajectory = rampWithStartAcceleration(model, 1.005);
  trajectory_processing::IterativeSplineParameterization parameterization(false);
  ASSERT_TRUE(parameterization.computeTimeStamps(trajectory));
  EXPECT_GT(trajectory.getDuration(), 0.0);
}

TEST(IterativeSplineParameterization, startStateClearlyOverLimitIsRejected) {
  auto model = makeModel();
  auto trajectory = rampWithStartAcceleration(model, 1.5);
  trajectory_processing::IterativeSplineParameterization parameterization(false);
  EXPECT_FALSE(parameterization.computeTimeStamps(trajectory));
}

namespace {

class FakeRobotModel : public hector_math::RobotModel<double>
{
public:
  explicit FakeRobotModel(std::vector<std::string> joints)
    : hector_math::RobotModel<double>(std::move(joints), {}) {}
  const double& mass() const override { return mass_; }
protected:
  hector_math::Vector3<double> computeCenterOfMass() const override { return hector_math::Vector3<double>::Zero(); }
  hector_math::Polygon<double> computeFootprint() const override { return {}; }
  Eigen::AlignedBox<double, 3> computeAxisAlignedBoundingBox() const override { return {}; }
private:
  double mass_ = 1.0;
};

// Records the joint positions of the robot model at every prediction and reports a stable square support polygon.
class RecordingPredictor : public hector_pose_prediction_interface::PosePredictor<double>
{
public:
  explicit RecordingPredictor(hector_math::RobotModel<double>::Ptr model) : model_(std::move(model)) {}
  hector_math::RobotModel<double>::Ptr robotModel() override { return model_; }
  hector_math::RobotModel<double>::ConstPtr robotModel() const override { return model_; }
  void updateSettings(const hector_pose_prediction_interface::PosePredictorSettings<double>& settings) override { settings_ = settings; }
  const hector_pose_prediction_interface::PosePredictorSettings<double>& settings() const override { return settings_; }

  mutable std::map<std::string, double> last_joints;

private:
  double doPredictPoseAndContactInformation(hector_math::Pose<double>&, hector_pose_prediction_interface::SupportPolygon<double>& polygon,
                                            hector_pose_prediction_interface::ContactInformation<double>&,
                                            hector_pose_prediction_interface::ContactInformationFlags,
                                            const hector_pose_prediction_interface::Wrench<double>&) const override
  {
    for (size_t i = 0; i < model_->jointNames().size(); ++i) {
      last_joints[model_->jointNames()[i]] = model_->jointPositions()[i];
    }
    square(polygon);
    return 1.0;
  }
  double doPredictPoseAndSupportPolygon(hector_math::Pose<double>&, hector_pose_prediction_interface::SupportPolygon<double>& polygon,
                                        const hector_pose_prediction_interface::Wrench<double>&) const override
  {
    square(polygon);
    return 1.0;
  }
  double doPredictPose(hector_math::Pose<double>&, const hector_pose_prediction_interface::Wrench<double>&) const override { return 1.0; }
  bool doEstimateSupportPolygon(const hector_math::Pose<double>&, hector_pose_prediction_interface::SupportPolygon<double>& polygon) const override
  {
    square(polygon);
    return true;
  }
  bool doEstimateContactInformation(const hector_math::Pose<double>&, hector_pose_prediction_interface::SupportPolygon<double>& polygon,
                                    hector_pose_prediction_interface::ContactInformation<double>&,
                                    hector_pose_prediction_interface::ContactInformationFlags) const override
  {
    square(polygon);
    return true;
  }
  static void square(hector_pose_prediction_interface::SupportPolygon<double>& polygon)
  {
    polygon.contact_hull_points = {{1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};
    polygon.edge_stabilities = {1.0, 1.0, 1.0, 1.0};
  }

  hector_math::RobotModel<double>::Ptr model_;
  hector_pose_prediction_interface::PosePredictorSettings<double> settings_{10, 0.01, 0.5};
};

}

TEST(FlipperMultiObjectiveCost, coupledJointsFollowTheirOptimizedJoint) {
  auto model = std::make_shared<FakeRobotModel>(std::vector<std::string>{"fl", "fr", "bl", "br"});
  auto predictor = std::make_shared<RecordingPredictor>(model);
  const std::vector<std::string> free_joints = {"fl", "bl"};
  const std::vector<std::string> coupled_joints = {"fr", "br"};
  FlipperMultiObjectiveCostParameters parameters;
  FlipperMultiObjectiveCost cost(free_joints, predictor, Eigen::Isometry3d::Identity(), nullptr, parameters,
                                 nullptr, coupled_joints);

  const double angles[2] = {0.3, -0.2};
  const double* blocks[1] = {angles};
  double value = 0.0;
  ASSERT_TRUE(cost(blocks, &value));
  EXPECT_DOUBLE_EQ(0.3, predictor->last_joints.at("fl"));
  EXPECT_DOUBLE_EQ(0.3, predictor->last_joints.at("fr"));
  EXPECT_DOUBLE_EQ(-0.2, predictor->last_joints.at("bl"));
  EXPECT_DOUBLE_EQ(-0.2, predictor->last_joints.at("br"));
}

namespace {

// One flipper joint and the planner's virtual joint
const char* kFlipperUrdf = R"(<robot name="flipper">
  <link name="base">
    <inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="flipper"/>
  <joint name="f" type="revolute">
    <parent link="base"/><child link="flipper"/><axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5" velocity="1.0" effort="1"/>
  </joint>
</robot>)";
const char* kFlipperSrdf = R"(<robot name="flipper">
  <virtual_joint name="world_virtual_joint" type="floating" parent_frame="world" child_link="base"/>
</robot>)";

// Predictions succeed only within 0.02 rad of the angle `feasible`, which the coarse global search misses
class BandPredictor : public hector_pose_prediction_interface::PosePredictor<double>
{
public:
  BandPredictor(hector_math::RobotModel<double>::Ptr model, double feasible) : model_(std::move(model)), feasible_(feasible) {}
  hector_math::RobotModel<double>::Ptr robotModel() override { return model_; }
  hector_math::RobotModel<double>::ConstPtr robotModel() const override { return model_; }
  void updateSettings(const hector_pose_prediction_interface::PosePredictorSettings<double>& settings) override { settings_ = settings; }
  const hector_pose_prediction_interface::PosePredictorSettings<double>& settings() const override { return settings_; }

private:
  bool feasible() const { return std::abs(model_->jointPositions()[0] - feasible_) < 0.02; }
  double doPredictPoseAndContactInformation(hector_math::Pose<double>& pose, hector_pose_prediction_interface::SupportPolygon<double>& polygon,
                                            hector_pose_prediction_interface::ContactInformation<double>&,
                                            hector_pose_prediction_interface::ContactInformationFlags,
                                            const hector_pose_prediction_interface::Wrench<double>&) const override
  {
    square(polygon, pose.asTransform());
    return feasible() ? 1.0 : std::nan("");
  }
  double doPredictPoseAndSupportPolygon(hector_math::Pose<double>& pose, hector_pose_prediction_interface::SupportPolygon<double>& polygon,
                                        const hector_pose_prediction_interface::Wrench<double>&) const override
  {
    square(polygon, pose.asTransform());
    return feasible() ? 1.0 : std::nan("");
  }
  double doPredictPose(hector_math::Pose<double>&, const hector_pose_prediction_interface::Wrench<double>&) const override { return 1.0; }
  bool doEstimateSupportPolygon(const hector_math::Pose<double>& pose, hector_pose_prediction_interface::SupportPolygon<double>& polygon) const override
  {
    square(polygon, pose.asTransform());
    return true;
  }
  bool doEstimateContactInformation(const hector_math::Pose<double>& pose, hector_pose_prediction_interface::SupportPolygon<double>& polygon,
                                    hector_pose_prediction_interface::ContactInformation<double>&,
                                    hector_pose_prediction_interface::ContactInformationFlags) const override
  {
    square(polygon, pose.asTransform());
    return true;
  }
  static void square(hector_pose_prediction_interface::SupportPolygon<double>& polygon, const Eigen::Isometry3d& pose)
  {
    // Clockwise and below the centre of mass at the robot origin, so the transition check finds it stable
    polygon.contact_hull_points.clear();
    for (const Eigen::Vector3d corner : {Eigen::Vector3d(1, -1, -0.5), Eigen::Vector3d(-1, -1, -0.5), Eigen::Vector3d(-1, 1, -0.5), Eigen::Vector3d(1, 1, -0.5)}) {
      polygon.contact_hull_points.push_back(pose * corner);
    }
    polygon.edge_stabilities = {1.0, 1.0, 1.0, 1.0};
  }

  hector_math::RobotModel<double>::Ptr model_;
  double feasible_;
  hector_pose_prediction_interface::PosePredictorSettings<double> settings_{10, 0.01, 0.5};
};

rclcpp::NodeOptions flipperNodeOptions(bool local_solver)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"robot_description", kFlipperUrdf},
    {"robot_description_semantic", kFlipperSrdf},
    {"flipper_optimization.joints", std::vector<std::string>{"f"}},
    {"flipper_optimization.solver.algorithm", "GN_DIRECT"},
    {"flipper_optimization.solver.maxeval", 3},
    {"flipper_optimization.local_solver.enabled", local_solver},
    {"flipper_optimization.local_solver.algorithm", "LN_SBPLX"},
    {"flipper_optimization.local_solver.maxeval", 20},
    {"arm_optimization.enabled", false},
    {"default_joint_state.f", 0.0},
  });
  return options;
}

FlipperOptimizationResult optimizeFromPreviousPosture(bool local_solver, double previous_angle)
{
  auto node = std::make_shared<rclcpp::Node>("flipper_optimization_test", flipperNodeOptions(local_solver));
  auto robot_model = std::make_shared<RobotModel>(node);
  auto predictor = std::make_shared<BandPredictor>(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), previous_angle);
  FlipperOptimization optimization(node->create_sub_node("flipper_optimization"), robot_model, predictor);

  auto previous = std::make_shared<moveit::core::RobotState>(robot_model->getMoveitRobotModel());
  previous->setToDefaultValues();
  previous->setVariablePosition("f", previous_angle);
  previous->update();
  return optimization.optimize(Eigen::Isometry3d::Identity(), {previous_angle}, previous);
}

PostureOptimizationResult findPosture(const std::shared_ptr<BandPredictor>& predictor, const Eigen::Isometry3d& pose)
{
  auto node = std::make_shared<rclcpp::Node>("posture_optimization_test", flipperNodeOptions(true));
  WholeBodyPostureOptimization optimization(node, predictor, std::make_shared<sdf_contact_estimation::SdfModel>());
  return optimization.findOptimalPosture(pose, {0.0});
}

}

TEST(WholeBodyPostureOptimization, noFeasiblePostureFails) {
  // Outside the joint limits, so every evaluated posture fails
  auto predictor = std::make_shared<BandPredictor>(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), 5.0);
  EXPECT_FALSE(findPosture(predictor, Eigen::Isometry3d::Identity()).success);
}

TEST(FlipperOptimization, localSolverKeepsAFeasiblePreviousPosture) {
  const FlipperOptimizationResult result = optimizeFromPreviousPosture(true, 0.9);
  EXPECT_LT(result.final_cost, std::numeric_limits<double>::max());
  EXPECT_NEAR(0.9, result.joint_state[0], 0.02);
}

TEST(FlipperOptimization, globalSearchAloneMissesANarrowFeasibleBand) {
  const FlipperOptimizationResult result = optimizeFromPreviousPosture(false, 0.9);
  EXPECT_EQ(std::numeric_limits<double>::max(), result.final_cost);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
