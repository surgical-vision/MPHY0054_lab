#!/usr/bin/env python3

import rospy
import numpy as np

from lab03_example_srv.srv import point_rot, point_rotResponse

def handle_point_rotation(request):
    px = request.p.x
    py = request.p.y
    pz = request.p.z

    p =np.array([px, py, pz])

    qx = request.q.x
    qy = request.q.y
    qz = request.q.z
    qw = request.q.w

    response = point_rotResponse()

    qxx =  qx **2
    qyy =  qy **2
    qzz =  qz **2

    R = np.zeros((3, 3))

    R[0,0] = 1 - 2*qyy-qzz
    R[1,1] = 1 - 2*qxx-qzz
    R[2,2] = 1 - 2*qxx-qyy

    R[0,1] = 2*qx*qy + qz*qw
    R[0,2] = 2*qx*qz - qy*qw

    R[1,0] = 2*qx *qy - 2*qz*qw
    R[1,2] = 2*qy*qz + 2*qx*qw

    R[2,0] = 2*qx*qz + 2*qy*qw
    R[2,1] = 2*qy*qz - 2*qx*qw

    result = np.dot(R, p) #3x3 * 3x1 = 3x1
    
    response.out_p.x = result[0]
    response.out_p.y = result[1]
    response.out_p.z = result[2]

    return response




def rotation_point_service():
    rospy.init_node('rotate_point', anonymous=True)
    rospy.Service('rotate_pt', point_rot, handle_point_rotation)
    rospy.spin()

if __name__ == "__main__":
    rotation_point_service()