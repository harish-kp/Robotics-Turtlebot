#!/usr/bin/env python

import rospy, cv_bridge, cv2
from geometry_msgs.msg import Twist
from cylinder.msg import cylDataArray
from sensor_msgs.msg import Image
import math
import numpy as np
import os

class FindCylinder:
    def __init__(self):
        self.run = True
        self.static_distance = 0
	self.label = 0
        self.count = 0
	self.error = 0
	self.bridge = cv_bridge.CvBridge()
	self.lower_red = np.array([23, 71, 72])
        self.upper_red = np.array([104, 219, 158])

    def cylinder_callback(self, data):
        if(len(data.cylinders) > 0):
            if(data.cylinders[0].label == 5):
		self.label = data.cylinders[0].label
             	self.run = False
		self.static_distance = data.cylinders[0].Zrobot

    def img_callback(self, data):
	image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv_image, self.lower_red, self.upper_red)
        M = cv2.moments(mask)
	h, w, d = image.shape
	if M['m00'] > 0:
		cx = int(M['m10']/M['m00'])
		self.error = cx - w/2

findCylinder = FindCylinder()
rospy.init_node('cylinder_detector')
cmd_vel_publish = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
rospy.Subscriber('/cylinderTopic', cylDataArray, findCylinder.cylinder_callback)
rospy.Subscriber('camera/rgb/image_raw', Image, findCylinder.img_callback)
twist = Twist()

while True:
    if(findCylinder.run):
        twist.linear.x = 0
        twist.angular.z = 0.2
        cmd_vel_publish.publish(twist)
    else:
        requ_distance = findCylinder.static_distance
	while(abs(findCylinder.error) > 2):
		print(findCylinder.error)
		twist.linear.x = 0
		twist.angular.z = -float(findCylinder.error) / 350
		cmd_vel_publish.publish(twist)
	
        distance = 0
        t0 = rospy.Time.now().to_sec()
        while(abs(requ_distance - distance) > 0.3):
            twist.linear.x = 0.1
            twist.angular.z = 0
            cmd_vel_publish.publish(twist)
            t1 = rospy.Time.now().to_sec()
            dt = t1 - t0
            distance = twist.linear.x * dt
        break
os.system("spd-say \' Found cylinder"+  str(findCylinder.label) + ' at ' + str(round(requ_distance, 2)) + "\'")

rospy.spin()
