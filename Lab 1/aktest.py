#!/usr/bin/env python


import time
import rospy

from create_node.msg import TurtlebotSensorState
from geometry_msgs.msg import Twist

import sys, select, termios, tty

flag = False
msg = """
Control Your Turtlebot!
---------------------------
j/l - nudges the bot in left/right
k/space - stops the bot for a while
"""

left_right_move = {'j':(1,0.5),'l':(1,-0.5)}
speed_delta = {'i':(1.1,1),'m':(.9,1),}

#Definitions
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def callBack(data):
    if(data.bumps_wheeldrops > 0):
        global flag
        flag = True
        print("Bump detected")
    else:
        flag = False
        
def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

speed = .2
turn = 1

if __name__=="__main__":
    
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5) #navigation definitions
    rospy.Subscriber('/mobile_base/sensors/core',TurtlebotSensorState, callBack) #bump sensor defintions

    linear_x = 0
    theta = 0
    try:
        print msg

        while(1):
            if(flag == False):
                print('inside 1st if')
                linear_x = 1
                theta = 0
                key = getKey()
            # nudging
            if key in left_right_move.keys():
                z = left_right_move[key][0]
                theta = left_right_move[key][1]
		    # pausing 
            if (key == ' ' )or (key == 'k') or (flag==True):
                linear_x = 0
                theta = 0
                time.sleep(10)
            #change of speed
            if (key == 'i' or key == 'm'):
                speed = speed * speed_delta[key][0]
                turn = turn * speed_delta[key][1]
		    #stopping with ctrl+c
            else:
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = speed * linear_x; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn * theta
            pub.publish(twist)


    except Exception as e:
        print ("Exception error occured")

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
