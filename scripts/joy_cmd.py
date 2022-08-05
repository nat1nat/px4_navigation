#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class joyObj:
    def __init__(self):
        print("Init Joy object.")
        rospy.Subscriber("/joy", Joy, self.joyCb)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.vel_data = Twist()
        self.lb = False

    def joyCb(self, msg):
        if (msg.buttons[4] == 1):
            self.lb = True
        else: self.lb = False
        self.vel_data.linear.x = msg.axes[1]*0.5
        self.vel_data.linear.y = msg.axes[0]*0.5
        self.vel_data.angular.z = msg.axes[3]*0.5

    def pubVel(self):
        if (not self.lb):
            self.vel_data.linear.x = 0.0
            self.vel_data.linear.y = 0.0
            self.vel_data.angular.z = 0.0
        self.vel_pub.publish(self.vel_data)

if __name__ == '__main__':
    rospy.init_node('joy_cmd')
    print("Initialize joy_cmd node!")

    joy = joyObj()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        joy.pubVel()
        rate.sleep()