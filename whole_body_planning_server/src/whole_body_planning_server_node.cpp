#include <whole_body_planning_server/whole_body_planning_server.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("whole_body_planning_server");
  try {
    whole_body_planning_server::WholeBodyPlanningServer planning_server(node);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    // Shut down while the robot model loaded.
    if (rclcpp::ok()) throw;
    RCLCPP_INFO(node->get_logger(), "Shut down during startup: %s", e.what());
    return 0;
  }
  rclcpp::shutdown();
  return 0;
}
