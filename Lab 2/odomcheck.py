#!/usr/bin/env python

import rospy

# from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class OdomCheck():
    def __init__(self):
        #f  = open("output2.txt", "w+")
        rospy.init_node('move_forward1')    
        self.xwrite = 0
        self.zwrite = 0
        self.time_init = 0
        self.time_last = 0
        rospy.Subscriber('/odom', Odometry, self.callback)
        rospy.spin()

    def callback(self, msg):        
        self.var = msg.twist.twist
        self.time_init = rospy.Time.now().to_sec()
        self.time_difference = self.time_init - self.time_last
        self.xwrite = self.var.linear.x * self.time_difference + self.xwrite
        self.zwrite = self.var.angular.z * self.time_difference + self.zwrite
        f.write('Relative Position at ' + str(self.time_init) + str(self.xwrite) + '\n')
        f.write('Relative Angle at ' + str(self.time_init) + str(self.zwrite) + '\n')
        print self.var
        if(self.xwrite == 2):
            f.write('\nTurtlebot crossed 2 metres\n')
        self.time_last = self.time_init
        # if(xwrite > 2 and var.linear.x <= 5.14):
        #     f.write('Turtlebot is taking semicircle')
        #if(var.linear.x > 5.14 and var.angular.z > 1.50):        

if __name__=="__main__":
    f  = open("output2.txt", "w+")
    OdomCheck()

