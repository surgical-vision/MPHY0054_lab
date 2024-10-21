#! /usr/bin/env python3

import rospy
import numpy as np
import random
import time

#TODO: import the SRV file from its corresponding folder, as well as its Request
from lab03_task.srv import rotmat2quat, rotmat2quatRequest

def rot2quat_client():
    rospy.wait_for_service('rotmat2quat')

    while not rospy.is_shutdown():
        #TODO: Initialise the ROS service client. It takes two arguments: The name of the service, and the service definition.
        client = rospy.ServiceProxy('rotmat2quat', rotmat2quat)

        #TODO: create a random request matrix
        request = rotmat2quatRequest()

        while not rospy.is_shutdown():
            request.R.data = np.random.uniform(-1,1,9)
            response = client(request)

            print(response)

            time.sleep(3)

if __name__ == '__main__':
    try:
        rot2quat_client()
    except rospy.ROSInterruptException:
        pass
