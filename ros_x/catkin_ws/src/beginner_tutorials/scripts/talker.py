#!/usr/bin/env python
# From http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# Publishes information to a ROS topic.

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher( 'chatter',
                           String,
                           queue_size = 10 )

    rospy.init_node( 'talker',
                     anonymous = True )

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        hello_str = "hello ros %s" % rospy.get_time()
        rospy.loginfo( hello_str )
        pub.publish( hello_str )
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
