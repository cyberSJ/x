#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
    ros::init( argc,
               argv,
               "basic_shapes" );

    ros::NodeHandle n;
    ros::Rate rate(1);

    ros::Publisher marker_pub = 
        n.advertise<visualization_msgs::Marker>(
            "visualization_marker", 
            1 );

    uint32_t shape = visualization_msgs::Marker::CUBE;

    while( ros::ok() )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shape";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        // While there are no subscribers, generate a warning message every 
        // second. But ROS_WARN_ONCE() will only generate the warning once.
        while( marker_pub.getNumSubscribers() < 1 )
        {
            // Poor-man version of making sure we can exit the program
            // gracefully.
            if( !ros::ok() )
            {
                return 0;
            }

            ROS_WARN_ONCE( "Please create a subscriber to the "
                           "visualization_marker" );
            
            // Instead of using ros sleep, use linux sleep.
            sleep(1);
        }

        marker_pub.publish( marker );

        // Choose the next shape to show.
        switch( shape )
        {
            case visualization_msgs::Marker::CUBE:
                shape = visualization_msgs::Marker::SPHERE;
                break;
            case visualization_msgs::Marker::SPHERE:
                shape = visualization_msgs::Marker::ARROW;
                break;
            case visualization_msgs::Marker::ARROW:
                shape = visualization_msgs::Marker::CYLINDER;
                break;
            case visualization_msgs::Marker::CYLINDER:
                shape = visualization_msgs::Marker::CUBE;
                break;
        }

        rate.sleep();
    }
}
