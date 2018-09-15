#!/usr/bin/env python

import rospy									
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys, select, termios, tty

g_last_twist = None
	
if __name__ == '__main__':	
	speed=0.2										#initializing motion variables
	turn=5
	d=0
	rospy.init_node('keys_to_twist')							#initializing node
	twist_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)		
	while not rospy.is_shutdown():
		t0 = rospy.get_time()
		while (d <= 2):
        		g_last_twist = Twist()
        		g_last_twist.linear.x = speed
        		twist_pub.publish(g_last_twist)
			tt = rospy.get_time()
			dt = tt - t0
			d = speed*dt
		rospy.sleep(0.5) 
		g_last_twist.angular.z = turn
        	twist_pub.publish(g_last_twist)
		rospy.sleep(0.5)
        	t0 = rospy.get_time()
		d=0
		while (d <= 2):
        		g_last_twist = Twist()
        		g_last_twist.linear.x = speed
        		twist_pub.publish(g_last_twist)
			tt = rospy.get_time()
			dt = tt - t0
			d = speed*dt
		rospy.sleep(0.5)
		t0 = rospy.get_time()
		d=0
		while (d <= 3.14):
        		g_last_twist = Twist()
        		g_last_twist.linear.x = speed
			g_last_twist.angular.z = 0.2
        		twist_pub.publish(g_last_twist)
			tt = rospy.get_time()
			dt = tt - t0
			d = speed*dt
		rospy.sleep(0.5)
		t0 = rospy.get_time()
		d=0
		while (d <= 2):
        		g_last_twist = Twist()
        		g_last_twist.linear.x = speed
        		twist_pub.publish(g_last_twist)
			tt = rospy.get_time()
			dt = tt - t0
			d = speed*dt
		rospy.sleep(0.5)
		g_last_twist.angular.z = turn
       		twist_pub.publish(g_last_twist)
		rospy.sleep(5)
		break
        	
            
	
