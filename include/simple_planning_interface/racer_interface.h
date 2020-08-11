#ifndef BASIC_INTERFACE_H
#define BASIC_INTERFACE_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>

#define MAX_X   5.5     //7
#define MAX_Y   5.5     //7
#define MAX_Z   3.0     //4

using namespace visualization_msgs;

class RacerInterface
{

 public:

  RacerInterface(ros::NodeHandle& nh_input);

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

  void cmdloopCallback(const ros::TimerEvent& event);

  Marker makeBox( InteractiveMarker &msg, float r, float g, float b );
  
  void buttonInitFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void buttonStartFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void buttonStopFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void makeInitButtonMarker( const tf::Vector3& position );
  void makeStartButtonMarker( const tf::Vector3& position );
  void makeStopButtonMarker( const tf::Vector3& position );
  
 private:
  // ROS publish variable
  ros::NodeHandle nh_;
  ros::Publisher init_publisher_;
  ros::Publisher run_publisher_;
  
};  // End of Class


#endif /* BASIC_INTERFACE_H */
