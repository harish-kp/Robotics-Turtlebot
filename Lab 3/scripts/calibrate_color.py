#!/usr/bin/env python

import cv2
import numpy as np
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image

class Follower:
    def __init__(self):
	    
			self.mask = 0
			self.bridge = cv_bridge.CvBridge()
			cv2.namedWindow('result')

			# Starting with 100's to prevent error while masking
			self.h, self.s, self.v = 100,100,100
			self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
									Image, self.image_callback)
			cv2.createTrackbar('h', 'result',0,179,self.nothing)
			cv2.createTrackbar('s', 'result',0,255,self.nothing)
			cv2.createTrackbar('v', 'result',0,255,self.nothing)

    def nothing(self):
        pass
    
    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
        self.h = cv2.getTrackbarPos('h','result')
        self.s = cv2.getTrackbarPos('s','result')
        self.v = cv2.getTrackbarPos('v','result')

        # Normal masking algorithm
        lower_blue = np.array([self.h,self.s,self.v])
        upper_blue = np.array([180,255,255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        result = cv2.bitwise_and(image, image, mask = mask)

        cv2.imshow('result', result)
                
		
               

rospy.init_node('line_follower')
follower = Follower()
while True:
	cv2.imshow('image', follower.mask)
	cv2.waitKey(0)
rospy.spin()
