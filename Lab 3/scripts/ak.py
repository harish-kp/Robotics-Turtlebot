#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from create_node.msg import TurtlebotSensorState
import sys, select, termios, tty

flag = False
bump_flag = False
class MoveForward():
    def __init__(self):
        rospy.init_node('move_forward')
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5)
        #rospy.Subscriber('/mobile_base/sensors/core',TurtlebotSensorState, callBack)
        self.mov_cmd = Twist()
    def go_forward(self, linear_speed):
        self.mov_cmd.linear.x = linear_speed
        self.mov_cmd.linear.y = 0
        self.mov_cmd.linear.z = 0
        self.pub.publish(self.mov_cmd)
        print('inside moving')
        #rospy.spin()
        #rate.sleep()
    def take_a_curve(self, linear_speed,angular_speed):
        self.mov_cmd.linear.x = linear_speed
        self.mov_cmd.linear.y = 0
        self.mov_cmd.linear.z = 0
	self.mov_cmd.angular.x = 0
        self.mov_cmd.angular.y = 0
        self.mov_cmd.angular.z = angular_speed
        self.pub.publish(self.mov_cmd)
        print('inside take a curve')
        #rospy.spin()
        #rate.sleep()
    
    def dont_move(self):
        self.twist = Twist()
        self.twist.linear.x = 0 
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.pub.publish(self.twist)
    def turn_right_angle(self):
        #clock_wise = input("Which direction need to turn?: ")
        self.mov_dist = Twist()
        self.mov_dist.linear.x = 0
        self.mov_dist.linear.y = 0
        self.mov_dist.linear.z = 0
        self.mov_dist.angular.x = 0
        self.mov_dist.angular.y = 0
       
        #if clock_wise:
        self.mov_dist.angular.z = -0.78
       
        #else:
            #self.mov_dist.angular.z = 0.78
   
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < 1.88):
           self.pub.publish(self.mov_dist)
           t1 = rospy.Time.now().to_sec()
           current_angle = 0.78*(t1-t0)
       
        self.mov_dist.angular.z = 0
        self.pub.publish(self.mov_dist)
        #rospy.spin()
        #rate.sleep()

def getKey():
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

if __name__=="__main__":
    move_forward = MoveForward()
    print('ON')
    flag = True
    distance = 0
    distance1 = 0
    linear_speed = 0.2
    angular_speed = -0.2
    t0 = rospy.Time.now().to_sec()
    t1 = 0
    try:
	#t0 = rospy.Time.now().to_sec()
	print('to' + str(t0))
        while(distance <= 2):
            print('before go forward')
            move_forward.go_forward(linear_speed)
            print('after go forward')
            t1 = rospy.Time.now().to_sec()
            distance = linear_speed *(t1-t0)
	    print(distance)
	t2 = t1
	while(distance1 <= 3.14285714300):
	    print('distance b4 start = ' + str(distance1))
            print('before takingh a curve')
            move_forward.take_a_curve(linear_speed,angular_speed)
            print('after taking a curve')
            t3 = rospy.Time.now().to_sec()
            distance1 = linear_speed *(t3-t2)
	    print(distance1)
	move_forward.correction()
    except Exception as e:
        print e
