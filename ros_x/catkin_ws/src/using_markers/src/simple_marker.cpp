#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

void processFeedback(
    const interactive_markers::
            InteractiveMarkerServer::
                FeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM( feedback->marker_name 
       << " is now at "
       << feedback->pose.position.x << ", " 
       << feedback->pose.position.y << ", "
       << feedback->pose.position.z );
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "simple_marker" );

    interactive_markers::InteractiveMarkerServer server("simple_marker");

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "my_interactive_marker";
    int_marker.description = "simple 1 dof control";

    // This just serves a purpose of indicating to human that there is a
    // physical interactive control that human eyes can recognize.
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    // So I'm gonna control the interactive marker using a CUBE?????
    // Maybe this defines how I'm going to control the cube....?
    // This just shows that there is a cube which "represents" the interactive
    // control.
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );

    int_marker.controls.push_back( box_control );

    visualization_msgs::InteractiveMarkerControl rotate_control;
    rotate_control.name = "move_along_x";
    rotate_control.interaction_mode = 
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    int_marker.controls.push_back( rotate_control );

    server.insert( int_marker, &processFeedback );
    server.applyChanges();
    
    // Enter a loop and call the call backs in this thread.
    ros::spin();
}
