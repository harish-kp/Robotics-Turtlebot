#!/usr/bin/env python


#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color blue to obtain the binary image
#to be able to see only the blue line and then follow that line
#It uses an approach called proportional and simply means
import cv2
import rospy, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time

class Follower:

        def __init__(self):
		self.mask = 0
                self.bridge = cv_bridge.CvBridge()
               
                self.result = 0
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                        Twist, queue_size=1)
                cv2.namedWindow('result')

                self.twist = Twist()
                cv2.createTrackbar('minh', 'result',0,179,self.nothing)
                cv2.createTrackbar('mins', 'result',0,255,self.nothing)
                cv2.createTrackbar('minv', 'result',0,255,self.nothing)
                cv2.createTrackbar('maxh', 'result',0,179,self.nothing)
                cv2.createTrackbar('maxs', 'result',0,255,self.nothing)
                cv2.createTrackbar('maxv', 'result',0,255,self.nothing)
		self.h, self.s, self.v = 100,100,100

        def nothing(self, hello):
                pass

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.min_h = cv2.getTrackbarPos('minh','result')
                self.min_s = cv2.getTrackbarPos('mins','result')
                self.min_v = cv2.getTrackbarPos('minv','result')
                self.max_h = cv2.getTrackbarPos('maxh','result')
                self.max_s = cv2.getTrackbarPos('maxs','result')
                self.max_v = cv2.getTrackbarPos('maxv','result')
                self.lower_blue = numpy.array([self.min_h, self.min_s, self.min_v])
                self.upper_blue = numpy.array([self.max_h, self.max_s, self.max_v])
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
                #ret, mask = cv2.threshold(selected_channel, 100, 240, cv2.THRESH_BINARY_INV)
                h, w, d = image.shape
                search_top = 3*h/4
                # search_bot = 3*h/4 + 20
                #mask[0:search_top, 0:w] = 0
                # M = cv2.moments(mask)
                result = cv2.bitwise_and(image, image, mask = mask)

                # cv2.imshow('result', follower.result)
                cv2.imshow('result', result)

		
		
                # if M['m00'] > 0:
                #         cx = int(M['m10']/M['m00'])
                #         cy = int(M['m01']/M['m00'])
                #         cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                #         #The proportional controller is implemented in the following four lines which
                #         #is reposible of linear scaling of an error to drive the control output.
                #         err = cx - w/2
                #         self.twist.linear.x = 0.1
                #         self.twist.angular.z = -float(err) / 100
                #         self.cmd_vel_pub.publish(self.twist)
		
                # else:
                #         self.twist.linear.x = 0
                #         self.twist.angular.z = 0.1
                #         self.cmdvel_pub.publish(self.twist)


                # cv2.waitKey(0)

rospy.init_node('line_follower')
follower = Follower()
while True:
        cv2.waitKey(0)
rospy.spin()
