#include <simple_planning_interface/basic_interface.h>

BasicInterface::BasicInterface(ros::NodeHandle& nh_input) 
:
nh_(nh_input)

{

  // create ros pose publisher
  pos_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("/planner_interface/desired_waypoints", 1);
  commit_publisher_ = nh_.advertise<std_msgs::Bool>("/planner_interface/commit", 1);

  // create a timer to update the published transforms
  // ros::Timer frame_timer = nh_.createTimer(ros::Duration(0.01), &BasicInterface::frameCallback, this);

  server.reset( new interactive_markers::InteractiveMarkerServer("basic_interface","",false) );

  // ros::Duration(0.1).sleep();


  tf::Vector3 position;

  position = tf::Vector3( 0, 0, 0);
  BasicInterface::makeQuadrocopterMarker( position );

  position = tf::Vector3( 0, 5, 0);
  BasicInterface::makeLoadButtonMarker( position );

  position = tf::Vector3( 2.5, 5, 0);
  BasicInterface::makeVisualizeButtonMarker( position );

  position = tf::Vector3( 5, 5, 0);
  BasicInterface::makeCommitButtonMarker( position );

  server->applyChanges();

}

// %Tag(Box)%
Marker BasicInterface::makeBox( InteractiveMarker &msg, float r, float g, float b )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  return marker;
}

Marker BasicInterface::makeArrow( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.2;
  marker.scale.z = msg.scale * 0.2;
  marker.color.r = 0.8;
  marker.color.g = 0.5;
  marker.color.b = 0.3;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& BasicInterface::makeArrowControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeArrow(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void BasicInterface::frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "map", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "map", "rotating_frame"));

  counter++;
}
// %EndTag(frameCallback)%

// Load button process
void BasicInterface::buttonLoadFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "load button right-clicked: Now load params from a yaml file" << std::endl;

    std::vector<double> waypoints;
    int num_points;
    geometry_msgs::PoseArray published_waypoints;

    if (nh_.getParam("/waypoints/data", waypoints) && nh_.getParam("/waypoints/num_points", num_points)){
      if ( waypoints.size() % 3 == 0 && waypoints.size() % 6 == 0){
        for (int i = 0; i < num_points; i ++){
          geometry_msgs::Pose pose;
          pose.position.x = waypoints[i*6];
          pose.position.y = waypoints[i*6+1];
          pose.position.z = waypoints[i*6+2];

          Eigen::Quaterniond q;
          q = Eigen::AngleAxisd(waypoints[i*6+5], Eigen::Vector3d::UnitZ())     //yaw
              * Eigen::AngleAxisd(waypoints[i*6+4], Eigen::Vector3d::UnitY())   //pitch
              * Eigen::AngleAxisd(waypoints[i*6+3], Eigen::Vector3d::UnitX());  //roll

          pose.orientation.x = q.x();
          pose.orientation.y = q.y();
          pose.orientation.z = q.z();
          pose.orientation.w = q.w();

          published_waypoints.poses.push_back(pose);
        }
      }
      else
        ROS_ERROR("Number of waypoints mismatched with data");

      pos_publisher_.publish(published_waypoints);
      std::cout << "params loaded and published" << std::endl;
    }
    else
      ROS_ERROR("Failed to get param file");
  }
  server->applyChanges();
}

// Visualize button process
void BasicInterface::buttonVisualizeFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "visualize button right-clicked: visualize trajectory" << std::endl;
    // Send non-commit request (visualize in RVIZ only) to local planner
    std_msgs::Bool bool_msg;
    bool_msg.data = false;
    commit_publisher_.publish(bool_msg);
  }
  server->applyChanges();
}

// Commit button process
void BasicInterface::buttonCommitFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "\033[1;33m commit button right-clicked: Now the robot will commit the trajectory\033[0m\n" << std::endl;
    // Send commit request to local planner
    std_msgs::Bool bool_msg;
    bool_msg.data = true;
    commit_publisher_.publish(bool_msg);
  }
  server->applyChanges();
}

// Drone Target moving feedback process
void BasicInterface::moveTargetQuadcopterFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE){

    if (feedback->pose.position.x <= MAX_X && feedback->pose.position.x >= -MAX_X 
          && feedback->pose.position.y <= MAX_Y && feedback->pose.position.y >= -MAX_Y 
          && feedback->pose.position.z <= MAX_Z && feedback->pose.position.z >= 0.3){
      
      Eigen::Quaterniond q;
      q.x() = feedback->pose.orientation.x;
      q.y() = feedback->pose.orientation.y;
      q.z() = feedback->pose.orientation.z;
      q.w() = feedback->pose.orientation.w;
      auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

      std::ostringstream s;
      ROS_INFO_STREAM( s.str() << ": Drone target moves. Press COMMIT button to for the drone to MOVE to:"
      << "\nposition = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << "\nyaw_angle = " << euler(2)
      << "\nframe: " << feedback->header.frame_id);

      geometry_msgs::PoseArray published_waypoints;
      geometry_msgs::Pose pose_msgs;

      pose_msgs.position.x = feedback->pose.position.x;
      pose_msgs.position.y = feedback->pose.position.y;
      pose_msgs.position.z = feedback->pose.position.z;

      pose_msgs.orientation.x = feedback->pose.orientation.x;
      pose_msgs.orientation.y = feedback->pose.orientation.y;
      pose_msgs.orientation.z = feedback->pose.orientation.z;
      pose_msgs.orientation.w = feedback->pose.orientation.w;

      published_waypoints.poses.push_back(pose_msgs);

      pos_publisher_.publish(published_waypoints);
      std::cout << "params loaded and published" << std::endl;
    }
    else{
      ROS_WARN("Drone cannot go out of the safety cage!");
    }
  }

  server->applyChanges();
}
////////////////////////////////////////////////////////////////////////////////////

// %Tag(Quadrocopter)%
void BasicInterface::makeQuadrocopterMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "quadrocopter";
  int_marker.description = "Quadrocopter";

  makeArrowControl(int_marker);

  InteractiveMarkerControl control;

  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&BasicInterface::moveTargetQuadcopterFeedback, this, _1));
}
// %EndTag(Quadrocopter)%

// Load Button
void BasicInterface::makeLoadButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "load_button";
  int_marker.description = "Load\n(from params)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker, 0.1, 0.7, 0.5);
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&BasicInterface::buttonLoadFeedback, this, _1));
}

// Visualize button
void BasicInterface::makeVisualizeButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "visualize_button";
  int_marker.description = "Visualize";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker, 0.3, 0.1, 0.8);
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&BasicInterface::buttonVisualizeFeedback, this, _1));
}

// Commit button
void BasicInterface::makeCommitButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "commit_button";
  int_marker.description = "Commit";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker, 0.9, 0.1, 0.1);
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&BasicInterface::buttonCommitFeedback, this, _1));
}
