#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_x/AveragingAction.h>
#include <boost/thread.hpp>

// Maybe this tutorial is just showing that you can spin a ros while running the
// action server.
void spinThread()
{
    ros::spin();
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "test_averaging" );

    actionlib::SimpleActionClient<
        actionlib_x::AveragingAction> ac( "averaging" );

    // sung try commenting out this line.
    boost::thread spin_thread( &spinThread );

    ROS_INFO( "Waiting for action server to start." );
    ac.waitForServer();

    ROS_INFO( "Action server started, sending goal." );
    actionlib_x::AveragingGoal goal;
    goal.samples = 100;
    ac.sendGoal( goal );

    bool finished_before_timeout = ac .waitForResult( ros::Duration(30.0) );

    if( finished_before_timeout )
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO( "Action finished: %s", state.toString().c_str() );
    }
    else
    {
        ROS_INFO( "Action did not finish before the timeout." );
    }

    ros::shutdown();
    spin_thread.join();

    return 0;
}
