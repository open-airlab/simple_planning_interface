#include <ros/ros.h>
#include <simple_planning_interface/inspection_mission_interface.h>

// Main node
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "planner_interface");
  ros::NodeHandle n;

  // Implement with a defined waypoints: start waypoint, land waypoint and a list of gate waypoints
  InspectionMissionInterface *inspection_mission_interface = new InspectionMissionInterface(n);
  

  ros::spin();

  inspection_mission_interface->server.reset();
  
  return 0;
}