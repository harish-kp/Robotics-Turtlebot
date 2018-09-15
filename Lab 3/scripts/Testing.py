#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

flag = False
class MoveForward():
    def __init__(self):
        rospy.init_node('move_forward')

        odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        odom_broadcaster = tf.TransformBroadcaster()

        self.sub_odom = rospy.Subscriber('/odom',Odometry, self.callback)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5)
        self.pub_pose = rospy.Publisher('cmd_vel_mux/input/navi', Pose, queue_size=5)
        self.mov_cmd = Twist()
        self.pose_cmd = Pose()
        self.rate = rospy.Rate(10)
        rospy.spin()
        #rospy.on_shutdown(self.shutdown)

    def callback(self, data):
        #if(data.pose.pose.x > 0):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        
        #    print(Pose.x)
        #    print(Pose.y)
        #    print(Pose)

        #    print(self.pose)
        #    self.pose.x = round(self.pose.x, 4)
        #    self.pose.y = round(self.pose.y, 4)
        print(self.pose.pose.pose)
        
        
            

    def go_forward(self, linear_speed):
        self.mov_cmd.linear.x = linear_speed
        self.mov_cmd.linear.y = 0
        self.mov_cmd.linear.z = 0
        self.pub.publish(self.mov_cmd)
        self.rate.sleep()
        print('inside moving')

if __name__=="__main__":
    move_forward = MoveForward()
    print('ON')
    flag = True
    distance = 0
    linear_speed = 0.2
    t0 = rospy.Time.now().to_sec()
    
    try:
        #while(flag):
        r = rospy.Rate(1.0)
        while(distance <= 200 and not rospy.is_shutdown()):
            print('before go forward')
            
            move_forward.go_forward(linear_speed)
            print('after go forward')
            print('time 1 = '+ str(t0))
            t1 = rospy.Time.now().to_sec()
            print('time 2 = '+ str(t1))
            distance = linear_speed *(t1)
            print('distance = '+ str(distance))
            r.sleep()

    except Exception as e:
        print e


