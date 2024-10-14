#!/usr/bin/env python

import rospy
from simple_msg_example.msg import CustomMsg

def msg_publisher():
    pub = rospy.Publisher('custom_topic', CustomMsg, queue_size=10)
    rospy.init_node('msg_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    id_counter = 0

    while not rospy.is_shutdown():
        custom_msg = CustomMsg()
        custom_msg.id = id_counter
        custom_msg.name = "example_message"
        custom_msg.value = 42.0 + id_counter
        rospy.loginfo(f"Publishing: id={custom_msg.id}, name={custom_msg.name}, value={custom_msg.value}")
        pub.publish(custom_msg)
        id_counter += 1
        rate.sleep()

if __name__ == "__main__":
    try:
        msg_publisher()
    except rospy.ROSInterruptException:
        pass
