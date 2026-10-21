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

double rollCost(double yaw, double pitch, double roll) {
  auto model = std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"});
  FlipperMultiObjectiveCostParameters parameters;
  parameters.impact_weight = 0.0;
  parameters.roll_cost_weight = 1.0;
  Eigen::Isometry3d pose(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  const std::vector<std::string> joints = {"f"};
  FlipperMultiObjectiveCost cost(joints, std::make_shared<RecordingPredictor>(model), pose, nullptr, parameters);
  const double angle[1] = {0.0};
  const double* blocks[1] = {angle};
  double value = 0.0;
  cost(blocks, &value);
  return value;
}

TEST(FlipperMultiObjectiveCost, rollCostPenalizesBodyRollAtAnyHeading) {
  EXPECT_NEAR(0.0, rollCost(M_PI_2, 0.5, 0.0), 1e-9);
  EXPECT_NEAR(0.09, rollCost(M_PI_2, 0.0, 0.3), 1e-9);
  EXPECT_NEAR(0.09, rollCost(0.8, 0.4, -0.3), 1e-9);
}

TEST(FlipperMultiObjectiveCost, symmetryCostPullsMirroredJointsTogether) {
  auto model = std::make_shared<FakeRobotModel>(std::vector<std::string>{"fl", "fr", "bl", "br"});
  FlipperMultiObjectiveCostParameters parameters;
  parameters.impact_weight = 0.0;
  parameters.lorentzian_sigma = 0.2;
  parameters.symmetry_weight = 2.0;
  const std::vector<std::string> joints = {"fl", "fr", "bl", "br"};
  FlipperMultiObjectiveCost cost(joints, std::make_shared<RecordingPredictor>(model), Eigen::Isometry3d::Identity(),
                                 nullptr, parameters, nullptr, {}, nullptr, {{0, 1}, {2, 3}});
  auto value = [&cost](std::vector<double> angles) {
    const double* blocks[1] = {angles.data()};
    double v = 0.0;
    cost(blocks, &v);
    return v;
  };
  EXPECT_NEAR(0.0, value({0.4, 0.4, -0.3, -0.3}), 1e-9);
  EXPECT_NEAR(2.0 * (lorentzian(0.5, 0.2) + lorentzian(0.1, 0.2)), value({0.4, -0.1, -0.3, -0.2}), 1e-9);
}

// Reports one track contact with the given surface normal and a stable square support polygon
class OneContactPredictor : public hector_pose_prediction_interface::PosePredictor<double>
{
public:
  OneContactPredictor(hector_math::RobotModel<double>::Ptr model, Eigen::Vector3d normal) : model_(std::move(model)), normal_(normal) {}
  hector_math::RobotModel<double>::Ptr robotModel() override { return model_; }
  hector_math::RobotModel<double>::ConstPtr robotModel() const override { return model_; }
  void updateSettings(const hector_pose_prediction_interface::PosePredictorSettings<double>& settings) override { settings_ = settings; }
  const hector_pose_prediction_interface::PosePredictorSettings<double>& settings() const override { return settings_; }

private:
  double doPredictPoseAndContactInformation(hector_math::Pose<double>&, hector_pose_prediction_interface::SupportPolygon<double>& polygon,
                                            hector_pose_prediction_interface::ContactInformation<double>& contacts,
                                            hector_pose_prediction_interface::ContactInformationFlags,
                                            const hector_pose_prediction_interface::Wrench<double>&) const override
  {
    polygon.contact_hull_points = {{1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};
    polygon.edge_stabilities = {1.0, 1.0, 1.0, 1.0};
    hector_pose_prediction_interface::ContactPointInformation<double> contact{};
    contact.link_type = hector_pose_prediction_interface::LinkType::Tracks;
    contact.surface_area = 0.01;
    contact.surface_normal = normal_;
    contacts.contact_points = {contact};
    return 1.0;
  }
  double doPredictPoseAndSupportPolygon(hector_math::Pose<double>&, hector_pose_prediction_interface::SupportPolygon<double>&,
                                        const hector_pose_prediction_interface::Wrench<double>&) const override { return 1.0; }
  double doPredictPose(hector_math::Pose<double>&, const hector_pose_prediction_interface::Wrench<double>&) const override { return 1.0; }
  bool doEstimateSupportPolygon(const hector_math::Pose<double>&, hector_pose_prediction_interface::SupportPolygon<double>&) const override { return true; }
  bool doEstimateContactInformation(const hector_math::Pose<double>&, hector_pose_prediction_interface::SupportPolygon<double>&,
                                    hector_pose_prediction_interface::ContactInformation<double>&,
                                    hector_pose_prediction_interface::ContactInformationFlags) const override { return true; }

  hector_math::RobotModel<double>::Ptr model_;
  Eigen::Vector3d normal_;
  hector_pose_prediction_interface::PosePredictorSettings<double> settings_{10, 0.01, 0.5};
};

double contactCost(const Eigen::Vector3d& normal, double yaw = 0.0) {
  auto model = std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"});
  FlipperMultiObjectiveCostParameters parameters;
  parameters.impact_weight = 0.0;
  parameters.track_contact_weight = 1.0;
  parameters.body_contact_weight = 10.0;
  const std::vector<std::string> joints = {"f"};
  const Eigen::Isometry3d pose(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  FlipperMultiObjectiveCost cost(joints, std::make_shared<OneContactPredictor>(model, normal.normalized()), pose, nullptr, parameters);
  const double angle[1] = {0.0};
  const double* blocks[1] = {angle};
  double value = 0.0;
  cost(blocks, &value);
  return value;
}

TEST(FlipperMultiObjectiveCost, contactsFromAboveOrBesideCostAsBodyContacts) {
  EXPECT_NEAR(-0.01, contactCost({0, 0, 1}), 1e-9);
  // A riser ahead is climbed, so it stays a track contact without traction
  EXPECT_NEAR(0.0, contactCost({-1, 0, 0}), 1e-9);
  EXPECT_NEAR(0.1, contactCost({0, 0, -1}), 1e-9);
  EXPECT_NEAR(0.1, contactCost({0, 1, 0}), 1e-9);
  EXPECT_NEAR(0.1, contactCost({0.2, -1, 0.2}), 1e-9);
  // Beside the robot in its frame, the normal is in the world frame
  EXPECT_NEAR(0.1, contactCost({1, 0, 0}, M_PI_2), 1e-9);
  EXPECT_NEAR(0.0, contactCost({0, 1, 0}, M_PI_2), 1e-9);
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
  /// Every predicted pose moves by this in the world
  Eigen::Vector3d settle = Eigen::Vector3d::Zero();
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
    pose = hector_math::Pose<double>(Eigen::Translation3d(settle) * pose.asTransform());
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

double costAfterStep(double step, FlipperMultiObjectiveCostParameters parameters, FlipperRejections* rejections = nullptr,
                     double angle = 0.0)
{
  auto node = std::make_shared<rclcpp::Node>("transition_test", flipperNodeOptions(true));
  auto robot_model = std::make_shared<RobotModel>(node);
  auto predictor = std::make_shared<BandPredictor>(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), angle);
  auto previous = std::make_shared<moveit::core::RobotState>(robot_model->getMoveitRobotModel());
  previous->setToDefaultValues();
  previous->setVariablePosition("f", angle);
  previous->update();
  // The support polygon reaches 1 m around the robot, so a step beyond it leaves the previous centre of mass outside
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = step;
  parameters.impact_weight = 0.0;
  const std::vector<std::string> joints = {"f"};
  FlipperMultiObjectiveCost cost(joints, predictor, pose, previous, parameters, nullptr, {}, rejections);
  const double angles[1] = {angle};
  const double* blocks[1] = {angles};
  double value = 0.0;
  cost(blocks, &value);
  return value;
}

TEST(FlipperMultiObjectiveCost, transitionRejectsAPolygonThatDropsThePreviousCentreOfMass) {
  FlipperMultiObjectiveCostParameters parameters;
  FlipperRejections rejections;
  EXPECT_EQ(std::numeric_limits<double>::max(), costAfterStep(1.5, parameters, &rejections));
  EXPECT_EQ(1, rejections.transition);
  EXPECT_LT(costAfterStep(0.5, parameters), std::numeric_limits<double>::max());
}

TEST(FlipperMultiObjectiveCost, aFailedPredictionCountsThePredictorStatus) {
  auto node = std::make_shared<rclcpp::Node>("diagnostics_test", flipperNodeOptions(true));
  auto robot_model = std::make_shared<RobotModel>(node);
  auto predictor = std::make_shared<BandPredictor>(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), 0.0);
  auto previous = std::make_shared<moveit::core::RobotState>(robot_model->getMoveitRobotModel());
  previous->setToDefaultValues();
  previous->update();
  sdf_contact_estimation::PredictionDiagnostics diagnostics;
  diagnostics.status = sdf_contact_estimation::PredictionStatus::FellOver;
  diagnostics.unknown_sampling_points = 3;
  FlipperRejections rejections;
  const std::vector<std::string> joints = {"f"};
  FlipperMultiObjectiveCost cost(joints, predictor, Eigen::Isometry3d::Identity(), previous, FlipperMultiObjectiveCostParameters(),
                                 nullptr, {}, &rejections, {}, &diagnostics);
  double value = 0.0;
  for (const double angle : {1.0, 0.0}) {
    const double angles[1] = {angle};
    const double* blocks[1] = {angles};
    cost(blocks, &value);
  }
  EXPECT_EQ(1, rejections.unsettled);
  EXPECT_EQ(1, rejections.fell_over);
  EXPECT_EQ(0, rejections.no_contact);
  EXPECT_EQ(2, rejections.touched_unknown);
  EXPECT_EQ(1, rejections.failed_touched_unknown);
}

TEST(FlipperMultiObjectiveCost, transitionWeightCostsATipInsteadOfRejectingIt) {
  FlipperMultiObjectiveCostParameters parameters;
  parameters.transition_weight = 2.0;
  const double small_tip = costAfterStep(1.5, parameters);
  const double large_tip = costAfterStep(2.5, parameters);
  EXPECT_LT(0.0, small_tip);
  EXPECT_LT(small_tip, large_tip);
  EXPECT_LT(large_tip, std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(0.0, costAfterStep(0.5, parameters));
}

TEST(WholeBodyPostureOptimization, aFeasiblePostureWithoutSurfaceColumnsSucceeds) {
  auto predictor = std::make_shared<BandPredictor>(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), 0.0);
  EXPECT_TRUE(findPosture(predictor, Eigen::Isometry3d::Identity()).success);
}

TEST(WholeBodyPostureOptimization, noFeasiblePostureFails) {
  // Outside the joint limits, so every evaluated posture fails
  auto predictor = std::make_shared<BandPredictor>(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), 5.0);
  EXPECT_FALSE(findPosture(predictor, Eigen::Isometry3d::Identity()).success);
}

TEST(FlipperMultiObjectiveCost, predictSettledPoseRejectsAPoseThatMovedTooFar) {
  BandPredictor predictor(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), 0.0);
  // 0.2 m in x, y and 0.36 m in 3D
  predictor.settle = Eigen::Vector3d(0.0, 0.2, -0.3);
  FlipperMultiObjectiveCostParameters parameters;
  hector_math::Pose<double> settled;
  hector_pose_prediction_interface::SupportPolygon<double> polygon;
  hector_pose_prediction_interface::ContactInformation<double> contacts;
  EXPECT_TRUE(std::isfinite(predictSettledPose(predictor, parameters, Eigen::Isometry3d::Identity(), settled, polygon, contacts)));
  parameters.max_position_deviation = 0.25;
  EXPECT_TRUE(std::isfinite(predictSettledPose(predictor, parameters, Eigen::Isometry3d::Identity(), settled, polygon, contacts)));
  parameters.max_position_deviation = 0.1;
  EXPECT_TRUE(std::isnan(predictSettledPose(predictor, parameters, Eigen::Isometry3d::Identity(), settled, polygon, contacts)));
  EXPECT_NEAR(0.2, settled.asTransform().translation().y(), 1e-9);
}

TEST(FlipperOptimization, aPostureThatSettlesTooFarAwayIsInfeasible) {
  rclcpp::NodeOptions options = flipperNodeOptions(true);
  options.append_parameter_override("flipper_optimization.cost_parameters.max_position_deviation", 0.1);
  auto node = std::make_shared<rclcpp::Node>("flipper_drift_test", options);
  auto robot_model = std::make_shared<RobotModel>(node);
  auto predictor = std::make_shared<BandPredictor>(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), 0.0);
  predictor->settle = Eigen::Vector3d(0.0, 0.2, 0.0);
  FlipperOptimization optimization(node->create_sub_node("flipper_optimization"), robot_model, predictor);
  EXPECT_EQ(std::numeric_limits<double>::max(), optimization.optimize(Eigen::Isometry3d::Identity(), {0.0}, nullptr).final_cost);
}

TEST(FlipperOptimization, mirroredJointsNeedOneKnownPartnerPerJoint) {
  for (const std::vector<std::string>& mirrored : {std::vector<std::string>{"f", "f"}, std::vector<std::string>{"g"}}) {
    rclcpp::NodeOptions options = flipperNodeOptions(true);
    options.append_parameter_override("flipper_optimization.mirrored_joints", mirrored);
    auto node = std::make_shared<rclcpp::Node>("mirrored_joints_test", options);
    auto robot_model = std::make_shared<RobotModel>(node);
    auto predictor = std::make_shared<BandPredictor>(std::make_shared<FakeRobotModel>(std::vector<std::string>{"f"}), 0.0);
    EXPECT_THROW(FlipperOptimization(node->create_sub_node("flipper_optimization"), robot_model, predictor), std::invalid_argument);
  }
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
