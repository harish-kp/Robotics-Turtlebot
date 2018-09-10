#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from create_node.msg import TurtlebotSensorState
import sys, select, termios, tty

class MoveForward():
    def __init__(self):
        rospy.init_node('move_forward')
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.mov_cmd = Twist()
	#global rate 
	#rate = rospy.Rate(0.05)

    def go_forward(self, linear_speed):
        self.mov_cmd.linear.x = linear_speed
        self.mov_cmd.linear.y = 0
        self.mov_cmd.linear.z = 0
        self.mov_cmd.angular.x = 0
        self.mov_cmd.angular.y = 0
        self.mov_cmd.angular.z = 0
        self.pub.publish(self.mov_cmd)

    def take_a_curve(self, linear_speed,angular_speed):
        self.mov_cmd.linear.x = linear_speed
        self.mov_cmd.linear.y = 0
        self.mov_cmd.linear.z = 0
        self.mov_cmd.angular.x = 0
        self.mov_cmd.angular.y = 0
        self.mov_cmd.angular.z = angular_speed
        self.pub.publish(self.mov_cmd)
    def take_a_curve2(self, linear_speed,angular_speed):
        self.mov_cmd.linear.x = linear_speed
        self.mov_cmd.linear.y = 0
        self.mov_cmd.linear.z = 0
        self.mov_cmd.angular.x = 0
        self.mov_cmd.angular.y = 0
        self.mov_cmd.angular.z = 0.2846152
        self.pub.publish(self.mov_cmd)
 
    # def correction(self):
    #     self.mov_cmd1 = Twist()
    #     self.mov_cmd1.linear.x = 0
    #     self.mov_cmd1.angular.z = -0.95
    #     self.pub.publish(self.mov_cmd1)

    def correction1(self):
        self.mov_cmd2 = Twist()
        self.mov_cmd2.linear.x = 0
        self.mov_cmd2.angular.z = -0.2
        self.pub.publish(self.mov_cmd2)

    def dont_move(self):
        self.twist = Twist()
        self.twist.linear.x = 0 
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.pub.publish(self.twist)

    def turn_right_angle(self):
        self.mov_dist = Twist()
        self.mov_dist.linear.x = 0
        self.mov_dist.linear.y = 0
        self.mov_dist.linear.z = 0
        self.mov_dist.angular.x = 0
        self.mov_dist.angular.y = 0
        self.mov_dist.angular.z = 0.665
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < 1.88):
           self.pub.publish(self.mov_dist)
           t1 = rospy.Time.now().to_sec()
           current_angle = 0.78*(t1-t0)
       
        self.mov_dist.angular.z = 0
        self.pub.publish(self.mov_dist)





if __name__=="__main__":
    move_forward = MoveForward()
    print('ON')
    distance = 0
    distance1 = 0
    linear_speed = 0.2
    angular_speed = 0.2
    
    t0 = rospy.Time.now().to_sec()
    t1 = 0
    t4 = 0
    t7 = 0
    t8 = 0
    t10 = 0
    t15 = 0
    try:      
        #t0 = rospy.Time.now().to_sec()
        #print('to' + str(t0))
        while(distance <= 1 and not rospy.is_shutdown()):
            #print('before go forward')
            move_forward.go_forward(linear_speed)
            #print('after go forward')
            t1 = rospy.Time.now().to_sec()
            distance = linear_speed *(t1-t0)        
        t2 = t1
        distance = 0
        while(distance <= 1.1 and not rospy.is_shutdown()):
                #print('before go forward')
                move_forward.go_forward(linear_speed)
                #print('after go forward')
                t9 = rospy.Time.now().to_sec()
                distance = linear_speed *(t9-t2)
                #print(distance)

        t3 = t9
        distance = 0
        move_forward.turn_right_angle()	
        while(distance <= 1 and not rospy.is_shutdown()):
            #print('before go forward')
            move_forward.go_forward(linear_speed)
            #print('after go forward')
            t5 = rospy.Time.now().to_sec()
            distance = linear_speed *(t5-t3)
            #print(distance)
        t6 = t5
        #print('distance = ' + str(distance1))	
        distance = 0

        while(distance <= 1.34 and not rospy.is_shutdown()):
            #print('before go forward')
            move_forward.go_forward(linear_speed)
            #print('after go forward')
            t15 = rospy.Time.now().to_sec()
            distance = linear_speed *(t15-t6)
            #print(distance)
        t16 = t15
        #print('distance = ' + str(distance1))	
        distance = 0

     	while(distance <= 1.57079633 and not rospy.is_shutdown()):
            #print('distance b4 start = ' + str(distance1))
            #print('before takingh a curve')
            move_forward.take_a_curve(linear_speed,angular_speed)
            #print('after taking a curve')
            t4 = rospy.Time.now().to_sec()
            distance = linear_speed *(t4-t16)
            #print(distance)
            
        #move_forward.correction()
        #rospy.sleep(0.5)        
        t14 = t4 
        distance = 0

        while(distance <= 1.58079633 and not rospy.is_shutdown()):
            #print('distance b4 start = ' + str(distance1))
            #print('before takingh a curve')
            move_forward.take_a_curve2(linear_speed,angular_speed)
            #print('after taking a curve')  
            t10 = rospy.Time.now().to_sec()
            distance = linear_speed *(t10-t14)
            #print(distance)
            
        #move_forward.correction()
        #rospy.sleep(0.5)        
        t15 = t10
        distance = 0

        while(distance <= 2.4 and not rospy.is_shutdown()):            
                move_forward.go_forward(linear_speed)
                t7 = rospy.Time.now().to_sec()
                distance = linear_speed *(t7-t15)
        move_forward.turn_right_angle()	
        #move_forward.correction1()


    except Exception as e:
        print e
