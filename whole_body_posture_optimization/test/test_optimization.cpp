#include <gtest/gtest.h>

#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost.h>
#include <whole_body_posture_optimization/util/iterative_spline_parameterization.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
