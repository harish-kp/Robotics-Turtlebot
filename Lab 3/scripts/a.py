#!/usr/bin/env python

import cv2
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time

class Follower:

    def __init__(self):
		self.mask = 0
                self.mask1 = 0
                self.bridge = cv_bridge.CvBridge()
               
                self.result = 0
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                        Twist, queue_size=1)
                cv2.namedWindow('result')
                self.lower_blue = np.array([99, 24, 44])
                self.upper_blue = np.array([139, 202, 253])

                self.lower_red = np.array([36, 214, 71])
                self.upper_red = np.array([179, 255, 255])

                self.twist = Twist()

    def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
                mask1 = cv2.inRange(hsv, self.lower_red, self.upper_red)
                #ret, mask = cv2.threshold(selected_channel, 100, 240, cv2.THRESH_BINARY_INV)
                h, w, d = image.shape

                mask = mask[0.9*h:h, :]
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                M = cv2.moments(mask)

                X = cv2.moments(mask)

                # cv2.imshow('result', follower.result)
                cv2.imshow('result', mask)
                if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                        #The proportional controller is implemented in the following four lines which
                        #is reposible of linear scaling of an error to drive the control output.
                        err = cx - w/2
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = -float(err) / 500
                        self.cmd_vel_pub.publish(self.twist)

                # elif (M['m00'] > 0 and X['m00'] > 0):
                #         dx = int(X['m10']/X['m00'])
                #         dy = int(X['m01']/X['m00'])
                #         cv2.circle(image, (dx, dy), 20, (0,0,255), -1)
                #         #The proportional controller is implemented in the following four lines which
                #         #is reposible of linear scaling of an error to drive the control output.
                #         err = dx - w/2
                #         self.twist.linear.x = 0
                #         self.twist.angular.z = 0
                #         self.cmd_vel_pub.publish(self.twist)
		
                # else:
                #         self.twist.linear.x = 0
                #         self.twist.angular.z = 0.2
                #         self.cmd_vel_pub.publish(self.twist)


rospy.init_node('line_follower')
follower = Follower()
while True:
    cv2.waitKey(0)
rospy.spin()