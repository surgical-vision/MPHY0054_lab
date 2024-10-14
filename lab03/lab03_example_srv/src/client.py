#! /usr/bin/env python3

import rospy
import numpy as np
import random
import time

#TODO: import the SRV file from its corresponding folder, as well as its Request
from lab03_example_srv.srv import point_rot, point_rotRequest

def point_rotation_client():
    rospy.wait_for_service('rotate_pt')
    client = rospy.ServiceProxy('rotate_pt', point_rot)
    request = point_rotRequest()

    while not rospy.is_shutdown():

        request.p.x = random.uniform(-2.0, 2.0)
        request.p.y = random.uniform(-2.0, 2.0)
        request.p.z = random.uniform(-2.0, 2.0)

        print(request)


        quaternion = np.random.rand(4)
        quaternion /= np.linalg.norm(quaternion)

        request.q.x = quaternion[0]
        request.q.y = quaternion[1]
        request.q.z = quaternion[2]
        request.q.w = quaternion[3]

        response = client(request)
        print(response)

        time.sleep(3)

if __name__ == '__main__':
    try:
        point_rotation_client()
    except rospy.ROSInterruptException:
        pass