#!/usr/bin/env python3

import rospy
from  std_msgs.msg import Float64MultiArray


def listen(msg):
    rospy.loginfo(msg.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', Float64MultiArray, callback=listen)

    rospy.spin()

if __name__ == '__main__':
    listener()