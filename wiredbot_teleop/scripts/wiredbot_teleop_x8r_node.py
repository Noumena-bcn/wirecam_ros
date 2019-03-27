#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

ch3_data = 0
_cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)


def move_cmd(direction="-"):

    rospy.init_node('wiredbot_teleop_x8r')
    _cmd_vel_sub = rospy.Subscriber('/ardunio', Float32MultiArray, callback_wiredbot_teleop)
    _twist_object = Twist()

    rate = rospy.Rate(2)
    if direction != "stop":
        while not rospy.is_shutdown():
            _twist_object.linear.x = ch3_data
            _cmd_vel_pub.publish(_twist_object)
            rate.sleep()
    else:
        _twist_object.linear.x = 0.0
        _cmd_vel_pub.publish(_twist_object)
        rate.sleep()


def callback_wiredbot_teleop(msg):
    global ch3_data
    ch3_data = msg.data[2]


if __name__ == '__main__':
    try:
        move_cmd()
    except rospy.ROSInterruptException:
        move_cmd('stop')
        pass
