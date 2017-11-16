#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_x/FibonacciAction.h>

// What could be problem?
// Symptom: The rostopic for the fibonacci is not being published. Also the 
// server node doesn't seem to be starting.
// 1. The server code does not publish the correct content.
// 2. There is an algoritm hang.
// 3. The binding is not good.
// 4. Node setup is not correct.


class FibonacciAction
{
    public:
        FibonacciAction( std::string name ) :
            as_( nh_,
                 name,
                 boost::bind(&FibonacciAction::executeCB, this, _1),
                 false ),
            action_name_( name )
        {
            as_.start();
        }

        ~FibonacciAction()
        {
        }

        // sung make this private.
        void executeCB( const actionlib_x::FibonacciGoalConstPtr &goal )
        {
            // Just for demo-purpose.
            ros::Rate r(1);

            bool success = true;

            feedback_.sequence.clear();
            feedback_.sequence.push_back(0);
            feedback_.sequence.push_back(1);

            ROS_INFO( "%s: Executing, creating fibonacci sequence of order %i "
                      "with seeds %i, %i", 
                      action_name_.c_str(),
                      goal->order,
                      feedback_.sequence[0],
                      feedback_.sequence[1] );

            for( int i = 1; i <= goal->order; ++i )
            {
                if( as_.isPreemptRequested() || !ros::ok() )
                {
                    ROS_INFO( "%s: Preempted", action_name_.c_str() );
                    as_.setPreempted();
                    success = false;
                    break;
                }

                feedback_.sequence.push_back( feedback_.sequence[i] +
                                              feedback_.sequence[i-1] );
                as_.publishFeedback( feedback_ );

                r.sleep();
            }

            if( success )
            {
                result_.sequence = feedback_.sequence;
                ROS_INFO( "%s: Succeeded", action_name_.c_str() );
                as_.setSucceeded( result_ );
            }
        }

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<actionlib_x::FibonacciAction> as_;
        std::string action_name_;
        actionlib_x::FibonacciFeedback feedback_;
        actionlib_x::FibonacciResult result_;
};

int main( int argc, char** argv )
{
    ROS_INFO( "hello actionlib" );
    ros::init( argc, argv, "fibonacci" );
    FibonacciAction fibonacci("fibonacci");
    ros::spin();
}
