#include <whole_body_planning_server/whole_body_planning_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_planning_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  whole_body_planning_server::WholeBodyPlanningServer driving_controller(nh, pnh);

  ros::spin();

  return 0;
}
