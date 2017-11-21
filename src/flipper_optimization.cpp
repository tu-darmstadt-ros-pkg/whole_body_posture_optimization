#include <whole_body_posture_optimization/flipper/flipper_optimization.h>

namespace whole_body_posture_optimization {

FlipperOptimization::FlipperOptimization(const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen,
                                         const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation)
  : robot_shape_gen_(robot_shape_gen), contact_estimation_(contact_estimation)
{
  ros::NodeHandle pnh("~/flipper");
  start_state_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("start_state", 10, true);
  result_state_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("result_state", 10, true);
  contact_points_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("contact_points", 10, true);
  result_com_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("result_com_pose", 10, true);
}

std::vector<double> FlipperOptimization::optimize(const Eigen::Affine3d &robot_pose, const Eigen::Affine3d &base_to_com, const std::vector<double>& init_position)
{
  ROS_INFO_STREAM("Starting flipper optimization");
  // Global, derivative-free
//  nlopt::opt opt(nlopt::GN_DIRECT_L,  robot_shape_gen_->getDof());
//    nlopt::opt opt(nlopt::GN_ORIG_DIRECT_L,  robot_shape_gen_->getDof());
//  nlopt::opt opt(nlopt::GN_CRS2_LM,  robot_shape_gen_->getDof()); // similar results to direct_l, 154
//  nlopt::opt opt(nlopt::GN_ISRES,  robot_shape_gen_->getDof()); // similar result to direct_l
//    nlopt::opt opt(nlopt::GN_ISRES,  robot_shape_gen_->getDof()); // similar result to direct_l

  // Local, derivative-free
//  nlopt::opt opt(nlopt::LN_COBYLA,  robot_shape_gen_->getDof()); // similar result to direct_l
    nlopt::opt opt(nlopt::LN_NELDERMEAD,  robot_shape_gen_->getDof()); // similar result to direct_l

  // Global, derivative
//  nlopt::opt opt(nlopt::GD_STOGO,  robot_shape_gen_->getDof()); // good result, but sloooow (3x compared to direct_l)


  // Local, derivative
//  nlopt::opt opt(nlopt::LD_LBFGS,  robot_shape_gen_->getDof()); // didn't work at all, only 0s

  // Define parameter bounds
  opt.set_lower_bounds(robot_shape_gen_->getLowerLimits());
  opt.set_upper_bounds(robot_shape_gen_->getUpperLimits());

  // Add objective function
  int verbosity_level = 1;
  CeresCostFunctionWrapper support_area_cost(ConvexHullAreaError::Create(robot_shape_gen_, contact_estimation_, robot_pose, base_to_com), verbosity_level);
  opt.set_min_objective(CeresCostFunctionWrapper::wrap, &support_area_cost);

  // Set convergence criterion
//  opt.set_xtol_rel(1e-4);
  opt.set_ftol_rel(1e-2);
//  opt.set_maxeval(50);
//  opt.set_stopval(-0.510003);
//  opt.set_ftol_abs(0.01);

  // Fill in initial joint state (from previous time step)
  std::vector<double> parameters = init_position;

  // Publish start state
  robot_shape_gen_->updateShape(init_position);
  sdf_contact_estimation::publishShape(robot_shape_gen_->getShape(), start_state_pub_, robot_pose, "world", Eigen::Vector3d(1, 0, 0));

  double minf;
  auto start = std::chrono::high_resolution_clock::now();
  nlopt::result nlopt_result;
  try {
    nlopt_result = opt.optimize(parameters, minf);
  }
  catch (std::invalid_argument e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }
  catch (nlopt::roundoff_limited e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }
  catch (std::runtime_error e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Optimization took " << elapsed.count()*1000.0 << " ms.");

  ROS_INFO_STREAM("Optimization returned " << resultToString(nlopt_result));
  ROS_INFO_STREAM("Result: " << vecToString(parameters));

  // Estimating again for debug output
  robot_shape_gen_->updateShape(parameters);
  sdf_contact_estimation::ContactEstimationResult result = contact_estimation_->estimateSupportPolygon(robot_pose, base_to_com);
  sdf_contact_estimation::publishShape(robot_shape_gen_->getShape(), result_state_pub_, result.estimated_pose, "world", Eigen::Vector3d(1, 1, 0));
  visualization_msgs::MarkerArray contact_points_array_msg;
  sdf_contact_estimation::createPointMarkerArray(result.contact_points, contact_points_array_msg, "world", "contact_points", Eigen::Vector3d(1, 0.5, 0), 0.03);
  sdf_contact_estimation::createPointMarkerArray(result.convex_hull_points, contact_points_array_msg, "world", "convex_hull", Eigen::Vector3d(0, 1, 0), 0.04);
  Eigen::Affine3d result_com_pose = result.estimated_pose * base_to_com;
  sdf_contact_estimation::publishPose(result_com_pub_, result_com_pose, "world");
  std::vector<double> stabilities = computeForceAngleStabilities<double>(result.convex_hull_points, Eigen::Vector3d(0, 0, -9.81), result_com_pose.translation());
  createConvexHullMarkerArray(result.convex_hull_points, stabilities, contact_points_array_msg);
  sdf_contact_estimation::deleteAllMarkers(contact_points_pub_);
  contact_points_pub_.publish(contact_points_array_msg);

  return parameters;
}

void FlipperOptimization::createConvexHullMarkerArray(const std::vector<Eigen::Vector3d>& points, const std::vector<double>& stabilities, visualization_msgs::MarkerArray &marker_array)
{
  for (unsigned i = 0; i < points.size(); i++) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.0;
    marker.scale.z = 0.00001;

    // calculate stability color
    double max_stability = 6.0;
    double clamped_stability = std::min(stabilities[i], max_stability);
    double stability_pct = clamped_stability/max_stability;
    marker.color.a = 1.0;
    marker.color.r = 1 - stability_pct;
    marker.color.g = stability_pct;
    marker.color.b = 0;

    marker.header.frame_id = "world";
    marker.ns = "support_polygon";
    marker.id = i;

    Eigen::Affine3d marker_pose = Eigen::Affine3d::Identity();
    tf::poseEigenToMsg(marker_pose, marker.pose);

    geometry_msgs::Point point_start;
    tf::pointEigenToMsg(points[i], point_start);
    marker.points.push_back(point_start);
    int ip1 = (i+1) % points.size(); // wrap around
    geometry_msgs::Point point_end;
    tf::pointEigenToMsg(points[ip1], point_end);
    marker.points.push_back(point_end);
    marker_array.markers.push_back(marker);
  }
}

}
