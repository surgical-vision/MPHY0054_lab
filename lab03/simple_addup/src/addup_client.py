#!/usr/bin/env python

import rospy
from simple_addup.srv import AddUp

def add_up_client(x, y):
    rospy.wait_for_service('add_up')
    try:
        add_up = rospy.ServiceProxy('add_up', AddUp)
        resp = add_up(x, y)
        if resp.success:
            rospy.loginfo(f"Success! The result is: {resp.result}")
        else:
            rospy.logwarn("Calculation failed. Mock error!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('addup_client')
    x = int(input("Enter first integer: "))
    y = int(input("Enter second integer: "))
    add_up_client(x, y)
