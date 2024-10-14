#!/usr/bin/env python

import rospy
import random
from simple_addup.srv import AddUp, AddUpResponse

def handle_add_up(req):
    success = random.choice([True, False])  # Mock success or failure
    result = req.a + req.b if success else 0
    rospy.loginfo(f"Returning result: {result} with success: {success}")
    return AddUpResponse(result=result, success=success)

def addup_server():
    rospy.init_node('addup_server')
    s = rospy.Service('add_up', AddUp, handle_add_up)
    rospy.loginfo("Ready to add up two integers.")
    rospy.spin()

if __name__ == "__main__":
    addup_server()
