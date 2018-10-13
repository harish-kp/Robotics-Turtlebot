#!/usr/bin/env python

import rospy

# from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt
class OdomCheck():
    def __init__(self):
        #f  = open("Measurement.txt", "w+")
        rospy.init_node('move_forward1')    
        self.xwrite = 0
        self.zwrite = 0
        self.array1 =[]
        self.array2=[]
        rospy.Subscriber('/odom', Odometry, self.callback)
        rospy.spin()

    def callback(self, msg):        
        self.var = msg.twist.twist

        self.xwrite = msg.pose.pose.position.x 
        self.zwrite = msg.pose.pose.position.y

        f.write(str(self.xwrite) +', ' + str(self.zwrite) + '\n')

        print self.var
        pub = rospy.Publisher('new_topic', Odometry, queue_size= 10)
        pub.publish(msg)
        self.array1.append(round(self.xwrite,2))
        self.array2.append(round(self.zwrite,2))
            

if __name__=="__main__":
    f  = open("measurement.txt", "w+")
    
    conn = OdomCheck()
    if(rospy.is_shutdown()):

        plt.plot(conn.array1,conn.array2)
        plt.show()
        rospy.signal_shutdown()
    
