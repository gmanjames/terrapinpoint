#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import rospy
import numpy as np
from sensor_msgs.msg import Image


def im_callback(im_dat):
    pass

def detector():
    rospy.init_node('terrapin_detector', anonymous=True)

    rospy.Subscriber('', Image, im_callback)

def run_detector_publisher():

    # Do a thing
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    detector()

    try:
        run_orange_publisher()
    except rospy.ROSInteruptException:
        pass

