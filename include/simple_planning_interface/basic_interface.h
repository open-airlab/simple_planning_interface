#ifndef BASIC_INTERFACE_H
#define BASIC_INTERFACE_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <geometry_msgs/PoseArray.h>

using namespace visualization_msgs;

class BasicInterface
{

 public:

  BasicInterface(ros::NodeHandle& nh_input);

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

  void cmdloopCallback(const ros::TimerEvent& event);

  Marker makeBox( InteractiveMarker &msg, float r, float g, float b );
  Marker makeArrow( InteractiveMarker &msg );
  InteractiveMarkerControl& makeArrowControl( InteractiveMarker &msg );
  void frameCallback(const ros::TimerEvent&);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void buttonLoadFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void buttonCommitFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void makeQuadrocopterMarker( const tf::Vector3& position );
  void makeLoadButtonMarker( const tf::Vector3& position );
  void makeCommitButtonMarker( const tf::Vector3& position );
  
 private:
  // ROS publish variable
  ros::NodeHandle nh_;
  ros::Publisher pos_publisher_;
  
};  // End of Class


#endif /* BASIC_INTERFACE_H */