#include <ros/ros.h>
#include <simple_planning_interface/basic_interface.h>

// Main node
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "planner_interface");
  ros::NodeHandle n;

  // Implement with a defined waypoints: start waypoint, land waypoint and a list of gate waypoints
  BasicInterface *basic_interface = new BasicInterface(n);
  

  ros::spin();

  basic_interface->server.reset();
  
  return 0;
}