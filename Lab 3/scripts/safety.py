#!/usr/bin/env python
import rospy
from create_node.msg import TurtlebotSensorState
from geometry_msgs.msg import Twist
import sys, select, termios, tty

DIRECTION_MAPPING = {
    'j': -1, # left
    'l': 1, # right
}

SPEED_MAPPING = {
    'i': 1, # increase speed
    'm': -1, # decrease speed
}

class KeyboardTeleop():
    def __init__(self):
        rospy.init_node('keyboard_teleop', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5) # navigation message publisher
        rospy.Subscriber('mobile_base/sensors/core', TurtlebotSensorState, self.on_sensor_event) # subsriber to all sensors in 'iRobot create'
        self.current_linear_speed = 0.5 
        self.should_stop = False # a flag to represent if robot should stop
        self.SPEED_FACTOR = 0.5 # increase linear speed by this factor
        
        self.move_cmd = Twist() # new message
    
    def on_shutdown(self): # ROS automatically calls this function if it shutsdown or is killed
        rospy.loginfo("Shutting down")
        self.cmd_pub.publish(Twist())
        rospy.sleep(1)

    def on_sensor_event(self, data): # callback for all sensors (see line 22)
        if(data.cliff_left or data.cliff_right or data.cliff_front_right or data.cliff_front_left or data.bumps_wheeldrops):
            print('********* CLiff or Bump *******************')
            self.should_stop = True # should stop if bumped or about to be cliffed
        else:
            self.should_stop = False

    def nudge(self, direction): 
        self.move_cmd.angular.x = direction * self.SPEED_FACTOR # just change direction and speed  of angular x 
        self.move_cmd.angular.y = 0
        self.move_cmd.angular.z = 0
        self.cmd_pub.publish(self.move_cmd) # send turn command to robot
        self.move_cmd.angular.x = 0 # set angular value back to zero to prevent continuous turning 
    
    def change_speed(self, multipler): # multiplier denotes increase (+1) or decrease (-1)
        self.current_linear_speed = self.current_linear_speed + (multipler * self.SPEED_FACTOR) # increase or decrease linear speed x 
    
    def move_forward(self):
        self.move_cmd.linear.x = self.current_linear_speed # set current speed 
        self.move_cmd.linear.y = 0
        self.move_cmd.linear.z = 0
        self.cmd_pub.publish(self.move_cmd) # keep moving forward
        
    def stop_robot(self):
        self.current_linear_speed = 0 # reset current speed
        self.move_cmd = Twist() # all values to zero
        self.cmd_pub.publish(self.move_cmd) # stop

def getKey(): # reads key press from the terminal
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


## program execution starts here


if __name__ == '__main__':
    kb_teleop = KeyboardTeleop() # create an instance of above Class
    rate = rospy.Rate(5)
    stop = False # flag to denote 
    while not rospy.is_shutdown():
        key = getKey() # read key press
        if(key == 'k' or key == '\x03' or kb_teleop.should_stop or stop): #stp if already stopped
            stop = True

        if(key == 'i'): # start moving if i is pressed even if stopped  
            stop = False

        if(stop):        
            kb_teleop.stop_robot()
        elif(key in DIRECTION_MAPPING.keys() and not kb_teleop.should_stop): # if not cliffed or bumped and key is j or l
            kb_teleop.nudge(DIRECTION_MAPPING[key]) # nudge to that direction
        elif(key in SPEED_MAPPING.keys() and not kb_teleop.should_stop): # if not cliffed or bumped and key is j or l
            kb_teleop.change_speed(SPEED_MAPPING[key]) # change speed based on i or m
        elif(not kb_teleop.should_stop):
            kb_teleop.move_forward() # otherwise keep moving forward based on set current speed
        rate.sleep()
    rospy.spin()

