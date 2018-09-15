#!/usr/bin/env python

import rospy

# from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
class OdomCheck():
    def __init__(self):
        #f  = open("output2.txt", "w+")
        rospy.init_node('move_forward1')    
        self.xwrite = 0
        self.zwrite = 0
        self.time_init = 0
        self.time_last = 0
        self.xcowrite = 0
        self.ycowrite = 0
        self.distancex = 0
        self.distancey = 0
         
        rospy.Subscriber('/odom', Odometry, self.callback)
        rospy.spin()

    def callback(self, msg):        
        self.var = msg.twist.twist

        self.xwrite = msg.pose.pose.position.x 
        self.zwrite = msg.pose.pose.position.y



        f.write(str(self.xwrite) +', ' + str(self.zwrite) + '\n')

        print self.var
        if(self.xwrite == 2):
            f.write('\nTurtlebot crossed 2 metres\n')
        self.time_last = self.time_init
        pub = rospy.Publisher('new_topic', Odometry, queue_size= 10)
        pub.publish(msg)
        
        

        # if(xwrite > 2 and var.linear.x <= 5.14):
        #     f.write('Turtlebot is taking semicircle')
        #if(var.linear.x > 5.14 and var.angular.z > 1.50):
            

if __name__=="__main__":
    f  = open("output2.txt", "w+")
    
    OdomCheck()

    
