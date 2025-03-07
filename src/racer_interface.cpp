#include <simple_planning_interface/racer_interface.h>

RacerInterface::RacerInterface(ros::NodeHandle& nh_input) 
:
nh_(nh_input)

{

  // command publisher
  init_publisher_ = nh_.advertise<std_msgs::Bool>("/racer_interface/init_status", 1);
  run_publisher_ = nh_.advertise<std_msgs::Bool>("/racer_interface/run_status", 1);
  emergency_publisher_ = nh_.advertise<std_msgs::Bool>("/racer_interface/emergency_status", 1);

  // create a timer to update the published transforms
  // ros::Timer frame_timer = nh_.createTimer(ros::Duration(0.01), &RacerInterface::frameCallback, this);

  server.reset( new interactive_markers::InteractiveMarkerServer("racer_interface","",false) );

  // ros::Duration(0.1).sleep();


  RacerInterface::makeInitButtonMarker( tf::Vector3( 0, 5.5, 0) );

  RacerInterface::makeStartButtonMarker( tf::Vector3( 5.5, 5.5, 0) );

  RacerInterface::makeStopButtonMarker( tf::Vector3( 5.5, -5.5, 0) );

  server->applyChanges();

}

// %Tag(Box)%
Marker RacerInterface::makeBox( InteractiveMarker &msg, float r, float g, float b )
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

// Init button process: get the drone to normal starting position
void RacerInterface::buttonInitFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "Init button right-clicked: Now drone will go to the designated starting point." << std::endl;

    std_msgs::Bool bool_msg;
    bool_msg.data = true;
    init_publisher_.publish(bool_msg);

  }
  server->applyChanges();
}

// Start button process: start autonomous navigation
void RacerInterface::buttonStartFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "Start button pressed. The autonomous algorithm is kicked in!" << std::endl;

    std_msgs::Bool bool_msg;
    bool_msg.data = true;
    run_publisher_.publish(bool_msg);
  }
  server->applyChanges();
}

// Stop button process: stop and land the drone
void RacerInterface::buttonStopFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK){
    std::cout <<  "\033[1;33m Stop button pressed. The robot will stop and land immediately!\033[0m\n" << std::endl;

    std_msgs::Bool bool_msg;
    bool_msg.data = false;
    run_publisher_.publish(bool_msg);
    init_publisher_.publish(bool_msg);
    
    bool_msg.data = true;
    emergency_publisher_.publish(bool_msg);
  }
  server->applyChanges();
}
////////////////////////////////////////////////////////////////////////////////////

// Init Button
void RacerInterface::makeInitButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "init_button";
  int_marker.description = "Initialize";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker, 0.1, 0.7, 0.5);
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&RacerInterface::buttonInitFeedback, this, _1));
}

// Start button
void RacerInterface::makeStartButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "start_button";
  int_marker.description = "Start";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker, 0.3, 0.1, 0.8);
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&RacerInterface::buttonStartFeedback, this, _1));
}

// Stop button
void RacerInterface::makeStopButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "stop_button";
  int_marker.description = "Stop";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker, 0.9, 0.1, 0.1);
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&RacerInterface::buttonStopFeedback, this, _1));
}
