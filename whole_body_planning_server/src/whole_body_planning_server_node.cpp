#include <whole_body_planning_server/whole_body_planning_server.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("whole_body_planning_server");

  whole_body_planning_server::WholeBodyPlanningServer planning_server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
