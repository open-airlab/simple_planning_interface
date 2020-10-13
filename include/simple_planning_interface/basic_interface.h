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

#define MAX_X   105.5     //7
#define MAX_Y   105.5     //7
#define MAX_Z   5.0     //4

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

  void buttonLoadFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void buttonVisualizeFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void buttonCommitFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void moveTargetQuadcopterFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void makeQuadrocopterMarker( const tf::Vector3& position );
  void makeLoadButtonMarker( const tf::Vector3& position );
  void makeVisualizeButtonMarker( const tf::Vector3& position );
  void makeCommitButtonMarker( const tf::Vector3& position );
  
 private:
  // ROS publish variable
  ros::NodeHandle nh_;
  ros::Publisher pos_publisher_;
  ros::Publisher commit_publisher_;
  
};  // End of Class


#endif /* BASIC_INTERFACE_H */
