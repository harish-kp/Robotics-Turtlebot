#!/usr/bin/env python

import rospy
#import time
# from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cylinder.msg import cylDataArray
from math import cos, sin
import matplotlib.pyplot as plt
class OdomCheck():
    def __init__(self):
        rospy.init_node('move_forward1')    
        self.xwrite = 0
        self.ywrite = 0
        self.dxwrite = 0
        self.dywrite = 0
        self.theta = 0
        self.dtheta = 0
        self.staticDepthDistance = 0
        self.staticHorizontalDistance = 0
        self.file1 =  open("measurement.txt", "w+")
        self.file2 =  open("trajectory_and_points.txt", "w+")

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/cylinderTopic', cylDataArray, self.cyl_callback)
        # rospy.spin()
        self.last_time = rospy.get_time()

    def cyl_callback(self, data):
        if(len(data.cylinders) > 0):
            self.label = data.cylinders[0].label
            self.staticDepthDistance = data.cylinders[0].Zrobot
            self.staticHorizontalDistance = data.cylinders[0].Xrobot


    def odom_callback(self, msg):        
        self.current_time = rospy.get_time()

        self.xwrite = msg.pose.pose.position.x 
        self.ywrite = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z

        dt = self.current_time - self.last_time

        self.file1.write(str(self.xwrite) +', ' + str(self.ywrite) + ', ' + str(self.theta) + ', '+
            str(self.dxwrite)+ ', ' + str(self.dywrite)+', '+ str(self.dtheta) +'\n')
	self.file2.write(str(self.staticDepthDistance)+', '+str(self.staticHorizontalDistance) + '\n')
            

if __name__=="__main__":
    conn = OdomCheck()
    rospy.spin()
