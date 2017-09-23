/*
 * Taken from
 * http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main( int argc,
          char** argv )
{
    ros::init( argc,
               argv,
               "talker" );

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>( "chatter",
                                                                1000 );

    ros::Rate loop_rate( 10 );

    int message_count = 0;

    while( ros::ok() )
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello ros" << message_count;
        msg.data = ss.str();

        ROS_INFO( "%s", msg.data.c_str() );

        chatter_pub.publish( msg );

        ros::spinOnce();

        loop_rate.sleep();

        ++message_count;
    }

    return 0;
}
