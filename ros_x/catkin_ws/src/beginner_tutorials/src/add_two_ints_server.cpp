#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

// ROS methods always return boolean? OR this specific function is designed like
// this?
bool add( beginner_tutorials::AddTwoInts::Request& req,
          beginner_tutorials::AddTwoInts::Response& res )
{
    res.sum = req.a + req.b;
    ROS_INFO( "request: x=%ld, y=%ld", (long int)req.a, (long int) req.b );
    ROS_INFO( "sending back response: z=%ld", (long int)res.sum );
    return true;
}

int main( int argc,
          char** argv )
{
    ros::init( argc,
               argv,
               "add_two_ints_server" );

    ros::NodeHandle n;

    // It is the node that advertises a service. Advertising a service requires
    // a name for the service and the C++ function for the service.
    ros::ServiceServer service = n.advertiseService( "add_two_ints", 
                                                     add );

    ROS_INFO( "Ready to add two ints." );

    // Blocking call.
    ros::spin();

    return 0;
}
