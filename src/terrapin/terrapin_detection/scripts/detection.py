#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


'''
Author: Garren Ijames

Detector.py: A class for processing depth image data
'''
class Detector:

    def __init__(self):

        # IDK what this is for
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.startup()


    def startup(self):
        rospy.init_node('terrapin_detector', anonymous=True)

        rospy.Subscriber('/camera/depth/image_raw', Image, self.process)

        rospy.spin()


    def process(self, im_dat):
        rospy.loginfo(im_dat.data)







def run_detector_publisher():

    # Do a thing
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    detector = Detector()

    #try:
    #    run_orange_publisher()
    #except rospy.ROSInteruptException:
    #    pass

