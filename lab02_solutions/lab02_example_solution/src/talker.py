#!/usr/bin/env python3

import rospy
from  std_msgs.msg import Float64MultiArray

def talker():
    rospy.init_node('talker', anonymous=True)

    pub = rospy.Publisher('chatter', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(10)

    msg = Float64MultiArray()
    msg.data = []

    i_count = 0

    while not rospy.is_shutdown():
        msg.data.append(i_count)
        i_count += 1
        pub.publish(msg)
        # print(msg.data)

        rospy.loginfo(msg.data)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    
    except rospy.ROSInterruptException:
        exit(1)

