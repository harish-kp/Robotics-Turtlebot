#!/usr/bin/env python

import cv2
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
import pygame
import os

class Follower:

    def __init__(self):
		self.mask = 0
                self.mask1 = 0
                self.bridge = cv_bridge.CvBridge()
               
                self.blue_result = 0
                self.red_result = 0
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                        Twist, queue_size=1)
                cv2.namedWindow('blue_result')
                cv2.namedWindow('red_result')
                self.lower_blue = np.array([93,46,203])
                self.upper_blue = np.array([179, 255, 255])

                self.lower_red = np.array([152, 148, 57])
                self.upper_red = np.array([179, 255, 255])

                self.twist = Twist()

    def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
                mask1 = cv2.inRange(hsv, self.lower_red, self.upper_red)
                h, w, d = image.shape

                mask = mask[0.9*h:h, :]
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                mask1 = mask1[0.9*h:h, :]
                mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel)
                mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, kernel)

                M = cv2.moments(mask)

                X = cv2.moments(mask1)

                cv2.imshow('blue_result', mask)
                cv2.imshow('red_result', mask1)
                
                if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                        err = cx - w/2
                        self.twist.linear.x = 0.25
                        self.twist.angular.z = -float(err) / 500
                        self.cmd_vel_pub.publish(self.twist)
                
                else:
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0.4
                        self.cmd_vel_pub.publish(self.twist)

                if (X['m00'] > 0):
                        dx = int(X['m10']/X['m00'])
                        dy = int(X['m01']/X['m00'])
                        cv2.circle(image, (dx, dy), 20, (0,0,255), -1)

                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
 
                        distance = 0
                        t0= rospy.Time.now().to_sec()
                        while(distance<=0.465):
                                self.twist.linear.x = 0.2
                                t1 = rospy.Time.now().to_sec()
                                distance = self.twist.linear.x * (t1-t0)
                                self.cmd_vel_pub.publish(self.twist)
                                self.twist = Twist()

                        pygame.init()
                        pygame.mixer.music.load('/home/aaromal/Desktop/tbm.mp3')
                        pygame.mixer.music.play()
                        while pygame.mixer.music.get_busy():
                                pass

                        rospy.signal_shutdown()

count = 0
while (count < 3):
        duration = 0.3 
        freq = 440
        os.system('play --no-show-progress --null --channels 1 synth %s sine %f' %(duration, freq))
        count = count+1

rospy.init_node('line_follower')
follower = Follower()

while True:
    cv2.waitKey(0)
rospy.spin()