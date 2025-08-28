#include <simple_planning_interface/basic_interface.h>

BasicInterface::BasicInterface(std::shared_ptr<rclcpp::Node> nh_input) 
:
nh_(nh_input)

{

  // Initialize server
  server = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "basic_interface_marker_server", nh_, false
  );

  // declare parameter for waypoints
  nh_->declare_parameter<std::vector<double>>("waypoints.data", std::vector<double>{});

  // create ros pose publisher
  pos_publisher_ = nh_->create_publisher<geometry_msgs::msg::PoseArray>("/planner_interface/desired_waypoints", 10);
  commit_publisher_ = nh_->create_publisher<std_msgs::msg::Bool>("/planner_interface/commit", 10);

  // Create the interface with buttons and quadcopter marker
  Eigen::Vector3d position1( 0, 0, 2.0);
  BasicInterface::makequadcopterMarker( position1 );

  Eigen::Vector3d position2( 0, 5, 0);
  BasicInterface::makeLoadButtonMarker( position2 );

  Eigen::Vector3d position3( 2.5, 5, 0);
  BasicInterface::makeVisualizeButtonMarker( position3 );

  Eigen::Vector3d position4( 5, 5, 0);
  BasicInterface::makeCommitButtonMarker( position4 );

}

void BasicInterface::testInteractiveMarker()
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.name = "test_marker";
  int_marker.description = "Interactive Marker Example";
  int_marker.pose.position.x = 0.0;
  int_marker.pose.position.y = 0.0;
  int_marker.pose.position.z = 1.0;

  // Add a simple cube control
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

  visualization_msgs::msg::Marker cube;
  cube.type = visualization_msgs::msg::Marker::CUBE;
  cube.scale.x = 0.45;
  cube.scale.y = 0.45;
  cube.scale.z = 0.45;
  cube.color.r = 0.0;
  cube.color.g = 1.0;
  cube.color.b = 0.0;
  cube.color.a = 1.0;

  control.markers.push_back(cube);
  int_marker.controls.push_back(control);

  server->insert(int_marker, std::bind(&BasicInterface::buttonCommitFeedback, this, std::placeholders::_1));
  server->applyChanges();
  std::cout << "test marker function" << std::endl;
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


// Load button process
void BasicInterface::buttonLoadFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "load button right-clicked: Now load params from a yaml file" << std::endl;

    std::vector<double> waypoints;
    int num_points;
    geometry_msgs::msg::PoseArray published_waypoints;

    //load params to waypoints
    if (nh_->get_parameter("waypoints.data", waypoints))
      std::cout << "Loaded waypoints with size: " << waypoints.size() << std::endl;
    else
      std::cout << "Failed to load waypoints." << std::endl;  
    
    num_points = waypoints.size() / 6; // each waypoint has 6 values: x,y,z,roll,pitch,yaw
    std::cout << "Number of waypoints: " << num_points << std::endl;

    if ( waypoints.size() % 3 == 0 && waypoints.size() % 6 == 0){
      for (int i = 0; i < num_points; i ++){
        geometry_msgs::msg::Pose pose;
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
      RCLCPP_ERROR(nh_->get_logger(), "Number of waypoints mismatched with data!");

    pos_publisher_->publish(published_waypoints);
    std::cout << "params loaded and published" << std::endl;
    }

  server->applyChanges();
}

// Visualize button process
void BasicInterface::buttonVisualizeFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "visualize button right-clicked: visualize trajectory" << std::endl;
    // Send non-commit request (visualize in RVIZ only) to local planner
    std_msgs::msg::Bool bool_msg;
    bool_msg.data = false;
    commit_publisher_->publish(bool_msg);
  }
  server->applyChanges();
}

// Commit button process
void BasicInterface::buttonCommitFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "\033[1;33m commit button right-clicked: Now the robot will commit the trajectory\033[0m\n" << std::endl;
    // Send commit request to local planner
    std_msgs::msg::Bool bool_msg;
    bool_msg.data = true;
    commit_publisher_->publish(bool_msg);
  }
  server->applyChanges();
}

// Drone Target moving feedback process
void BasicInterface::moveTargetQuadcopterFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE){

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
      RCLCPP_INFO_STREAM(nh_->get_logger(), s.str() << ": Drone target moves. Press COMMIT button to for the drone to MOVE to:"
      << "\nposition = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << "\nyaw_angle = " << euler(2)
      << "\nframe: " << feedback->header.frame_id);

      geometry_msgs::msg::Pose pose_msgs;
      geometry_msgs::msg::PoseArray moving_target_waypoints;

      pose_msgs.position.x = feedback->pose.position.x;
      pose_msgs.position.y = feedback->pose.position.y;
      pose_msgs.position.z = feedback->pose.position.z;

      pose_msgs.orientation.x = feedback->pose.orientation.x;
      pose_msgs.orientation.y = feedback->pose.orientation.y;
      pose_msgs.orientation.z = feedback->pose.orientation.z;
      pose_msgs.orientation.w = feedback->pose.orientation.w;

      moving_target_waypoints.poses.push_back(pose_msgs);
      pos_publisher_->publish(moving_target_waypoints);
      std::cout << "params loaded and published" << std::endl;

    }
    else{
      RCLCPP_WARN(nh_->get_logger(), "Warning: Drone cannot go out of the safety cage!");
    }
  }

  server->applyChanges();
}
////////////////////////////////////////////////////////////////////////////////////

// %Tag(quadcopter)%
void BasicInterface::makequadcopterMarker( const Eigen::Vector3d& position )
{
  InteractiveMarker int_marker;
  
  int_marker.header.frame_id = "map";
  setMarkerPosition(int_marker.pose, position);
  int_marker.scale = 1;

  int_marker.name = "quadcopter";
  int_marker.description = "quadcopter";

  makeArrowControl(int_marker);

  InteractiveMarkerControl control;

  Eigen::Quaterniond orien(1.0, 0.0, 1.0, 0.0);
  orien.normalize();
  // control.orientation = setMarkerOrientation(orien);
  control.orientation.x = orien.x();
  control.orientation.y = orien.y();
  control.orientation.z = orien.z();
  control.orientation.w = orien.w();

  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, std::bind(&BasicInterface::moveTargetQuadcopterFeedback, this, std::placeholders::_1));
  server->applyChanges();
}
// %EndTag(quadcopter)%

// Load Button
void BasicInterface::makeLoadButtonMarker( const Eigen::Vector3d& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  setMarkerPosition(int_marker.pose, position);
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
  server->setCallback(int_marker.name, std::bind(&BasicInterface::buttonLoadFeedback, this, std::placeholders::_1));
  server->applyChanges();
}

// Visualize button
void BasicInterface::makeVisualizeButtonMarker( const Eigen::Vector3d& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  setMarkerPosition(int_marker.pose, position);

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
  server->setCallback(int_marker.name, std::bind(&BasicInterface::buttonVisualizeFeedback, this, std::placeholders::_1));
  server->applyChanges();
}

// Commit button
void BasicInterface::makeCommitButtonMarker( const Eigen::Vector3d& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  // int_marker.header.stamp = ros::Time::now;
  setMarkerPosition(int_marker.pose, position);

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
  server->setCallback(int_marker.name, std::bind(&BasicInterface::buttonCommitFeedback, this, std::placeholders::_1));
  server->applyChanges();
}
