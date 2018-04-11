#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def run_publisher():
    pub1  = rospy.Publisher('/terrapin/velocity', String, queue_size=10)
    pub2  = rospy.Publisher('/terrapin/messages', String, queue_size=10)
    pub3  = rospy.Publisher('/terrapin/misc',     String, queue_size=10)

    rate = rospy.Rate(10)

    vel = "4"
    msg = "msg"
    mis = "misc"
    while not rospy.is_shutdown():
        pub1.publish(vel)
        pub2.publish(msg)
        pub3.publish(mis)

        rospy.loginfo('vel: {}, msg: {}, mis: {}'.format(vel, msg, mis))


def tests_spinoff():
    rospy.init_node('terrapin_tests', anonymous=True)


if __name__ == '__main__':
    tests_spinoff()

    try:
        run_publisher()
    except rospy.ROSInteruptException:
        pass
