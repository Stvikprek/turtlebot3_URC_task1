#!/usr/bin/python3

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
def detect(imgmsg):
    img = br.imgmsg_to_cv2(imgmsg, "bgr8")
    img = cv2.resize(img,(780,540),interpolation=cv2.INTER_LINEAR)
    corns,ids,rejects = detector.detectMarkers(img)
    print(ids)
    

if __name__ == '__main__':   
    rospy.init_node('aruco_detect')
    rospy.Subscriber('/camera/rgb/image_raw', sensor_msgs.msg.Image, detect)
    rospy.spin()
