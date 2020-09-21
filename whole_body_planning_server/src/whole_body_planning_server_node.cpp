#include <whole_body_planning_server/whole_body_planning_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_planning_server");
  ros::NodeHandle pnh("~");

  ros::spin();

  return 0;
}
