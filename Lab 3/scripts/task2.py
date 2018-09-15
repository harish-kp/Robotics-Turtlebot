#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from cylinder.msg import cylDataArray
import math

class FindCylinder:
    def __init__(self):
        self.run = True
        self.staticDistance = 0
        self.count = 0
    def callback(self, data):
        if(len(data.cylinders) > 0):
            if(data.cylinders[0].label == 5 and self.run):
                self.count += 1
                if (self.count > 3):
                    self.run = False
                    self.staticDistance = data.cylinders[0].Zrobot

findCylinder = FindCylinder()
rospy.init_node('cylinder_detector')
cmd_vel_publish = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
rospy.Subscriber('/cylinderTopic', cylDataArray, findCylinder.callback)
twist = Twist()
while True:
    if(findCylinder.run):
        twist.linear.x = 0
        twist.angular.z = 0.2
        cmd_vel_publish.publish(twist)
    else:
        
        requ_distance = findCylinder.staticDistance
        distance = 0
        t0 = rospy.Time.now().to_sec()
        while(abs(requ_distance - distance) > 0.3):
            twist.linear.x = 0.1
            twist.angular.z = 0
            cmd_vel_publish.publish(twist)
            t1 = rospy.Time.now().to_sec()
            dt = t1 - t0
            distance = twist.linear.x * dt
            print(requ_distance, distance, t0, t1)
        break


rospy.spin()