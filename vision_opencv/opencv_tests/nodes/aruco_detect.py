#!/usr/bin/python3
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
Updated: Copyright (c) 2016, Tal Regev.
"""

import sys
import os
from optparse import OptionParser

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy

br = CvBridge()
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)
def detect_and_draw(imgmsg):
    img = br.imgmsg_to_cv2(imgmsg, "bgr8")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corns,ids,rejects = detector.detectMarkers(gray)
    pub = rospy.Publisher('/img_lol',sensor_msgs.msg.Image,queue_size=100)
    new_img = br.cv2_to_imgmsg(gray,encoding="passthrough")
    pub.publish(new_img)
    print(ids)
    

if __name__ == '__main__':   
    rospy.init_node('aruco_detect')
    rospy.Subscriber('/camera/rgb/image_raw', sensor_msgs.msg.Image, detect_and_draw)
    rospy.spin()
