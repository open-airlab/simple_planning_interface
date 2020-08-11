#include <ros/ros.h>
#include <simple_planning_interface/racer_interface.h>

// Main node
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "racer_interface");
  ros::NodeHandle n;

  // Implement with a defined waypoints: start waypoint, land waypoint and a list of gate waypoints
  RacerInterface *racer_interface = new RacerInterface(n);
  

  ros::spin();

  racer_interface->server.reset();
  
  return 0;
}