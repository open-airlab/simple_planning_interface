#ifndef BASIC_INTERFACE_H
#define BASIC_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <math.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>

#define MAX_X   5.5     //7
#define MAX_Y   5.5     //7
#define MAX_Z   3.0     //4

using namespace visualization_msgs::msg;
class BasicInterface
{

 public:

  BasicInterface(std::shared_ptr<rclcpp::Node> nh_input);

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  

  Marker makeBox( InteractiveMarker &msg, float r, float g, float b );
  Marker makeArrow( InteractiveMarker &msg );
  InteractiveMarkerControl& makeArrowControl( InteractiveMarker &msg );

  void buttonLoadFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void buttonVisualizeFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void buttonCommitFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void moveTargetQuadcopterFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void makequadcopterMarker( const Eigen::Vector3d& position );
  void makeLoadButtonMarker( const Eigen::Vector3d& position );
  void makeVisualizeButtonMarker( const Eigen::Vector3d& position );
  void makeCommitButtonMarker( const Eigen::Vector3d& position );
  
 private:
  // ROS publish variable
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pos_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr commit_publisher_;

  void setMarkerPosition(geometry_msgs::msg::Pose& pose, const Eigen::Vector3d& position)
  {
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
  }

  
};  // End of Class


#endif /* BASIC_INTERFACE_H */
