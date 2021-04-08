#!/usr/bin/env python3

"""
SUBSCRIBES TO:
    /robot/camera1/image_raw 
    this is the topic that the robot is listening to communicate with the camera. In order to get the 
    image you would have to do something along the lines of:
    self.image_sub = rospy.Subscriber("robot/camera1/image_raw", Image, self.callback)
PUBLISHES TO:
    /robot/name_of_topic
    you can change this to whatever you want. Here you will publish data for the control team to interpret
    and use for steering. To publish something you do:
    self.name_pub.publish(self.name_of_topic)

"""
#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import sys
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.point_detector import *

added in new line
