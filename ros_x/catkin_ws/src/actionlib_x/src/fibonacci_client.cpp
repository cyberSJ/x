#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_x/FibonacciAction.h>

int main( int argc, char** argv )
{
    ros::init( argc, argv, "test_fibonacci" );

    actionlib::SimpleActionClient<actionlib_x::FibonacciAction> ac( "fibonacci",
                                                                    true );

    ROS_INFO( "Waiting for action server to start." );
    ac.waitForServer();

    ROS_INFO( "Action server started, sending goal." );
    actionlib_x::FibonacciGoal goal;
    goal.order = 20;
    ac.sendGoal( goal );

    bool finished_before_timeout = ac.waitForResult( ros::Duration(30.0) );

    if( finished_before_timeout )
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO( "Action finished: %s", 
                  state.toString().c_str() );
    }
    else
    {
        ROS_WARN( "Action did not finish before the timeout." );
    }

    return 0;
}
