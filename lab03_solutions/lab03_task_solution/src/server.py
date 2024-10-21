#!/usr/bin/env python3

import rospy
import numpy as np

# TODO: Include all the required service classes
from lab03_task.srv import rotmat2quat, rotmat2quatResponse

def convert_rotmat2quat(request):
    """Callback ROS service function to convert a rotation matrix into a quaternion

    Args:
        request (rotmat2quatRequest): lab03_task service message, containing
        the rotation matrix you need to convert.

    Returns:
        rotmat2quatResponse: lab03_task service response, in which 
        you store the requested quaternion
    """

    # TODO: complete the function to transform a rotation matrix to quaternion
    rotation_matrix = np.array(request.R.data).reshape(3,3)
    trace = np.trace(rotation_matrix)
    theta = np.arccos((trace-1)/2)

    response = rotmat2quatResponse()

    if theta == 0:
        response.q.x = 0
        response.q.y = 0
        response.q.z = 0
        response.q.w = 1
    elif theta == np.pi or theta == -np.pi:
        K = 1/2 * (rotation_matrix + np.eye(3))
        sth2 = np.sin(theta/2)
        response.q.x = np.sqrt(K[0,0])*sth2
        response.q.y = np.sqrt(K[1,1])*sth2
        response.q.z = np.sqrt(K[2,2])*sth2
        response.q.w = 0
    else:
        sth = np.sin(theta)
        sth2 = np.sin(theta/2)
        cth2 = np.cos(theta/2)

        rx = 1/(2*sth) * (rotation_matrix[2, 1] - rotation_matrix[1, 2])
        ry = 1/(2*sth) * (rotation_matrix[0, 2] - rotation_matrix[2, 0])
        rz = 1/(2*sth) * (rotation_matrix[1, 0] - rotation_matrix[0, 1])

        response.q.x = rx * sth2
        response.q.y = ry * sth2
        response.q.z = rz * sth2
        response.q.w = cth2
    
    return response


def rotation_converter():
    rospy.init_node('rotation_converter')

    # TODO: Initialise the service
    rospy.Service('rotmat2quat', rotmat2quat, convert_rotmat2quat)
    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
