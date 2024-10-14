#!/usr/bin/env python

import rospy
from simple_msg_example.msg import CustomMsg

def callback(data):
    rospy.loginfo(f"Received: id={data.id}, name={data.name}, value={data.value}")

def msg_subscriber():
    rospy.init_node('msg_subscriber', anonymous=True)
    rospy.Subscriber('custom_topic', CustomMsg, callback)
    rospy.spin()

if __name__ == "__main__":
    msg_subscriber()
