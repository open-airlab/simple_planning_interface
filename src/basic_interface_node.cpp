#include <simple_planning_interface/basic_interface.h>

// Main node
int main (int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create a ROS2 node
  auto node = std::make_shared<rclcpp::Node>("my_node");

  // Pass the node to your class
  auto basic_interface_node = std::make_shared<BasicInterface>(node); 

  basic_interface_node->server.reset();

  // Keep node alive and processing callbacks
  rclcpp::spin(node);

  // Optional cleanup
  basic_interface_node->server.reset();
  rclcpp::shutdown();
  
  return 0;
}