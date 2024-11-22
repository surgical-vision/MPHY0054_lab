# Import necessary ROS and Python modules
import rosbag
import rospy
from std_msgs.msg import String, Int32

# Example: Writing to a ROS bag
bag = rosbag.Bag('example_write.bag', 'w')
try:
    # Open the bag in write mode and write some messages
    rospy.loginfo("Writing messages to the ROS bag...")
    bag.write('chatter', String(data='Hello, ROS!'))
    bag.write('numbers', Int32(data=42))
finally:
    bag.close()
    rospy.loginfo("Finished writing to the ROS bag.")
# Example: Reading from a ROS bag
print("\nReading messages from the ROS bag:")
bag = rosbag.Bag('example_write.bag', 'r')
try:
    for topic, msg, t in bag.read_messages():
        print(f"Topic: {topic}, Message: {msg}, Timestamp: {t}")
finally:
    bag.close()
# Example: A simple function to calculate the sum of numbers from a ROS bag
def calculate_sum_from_bag(bag_name, topic_name):
    total = 0
    try:
        bag = rosbag.Bag(bag_name, 'r')
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            if isinstance(msg, Int32):
                total += msg.data
    finally:
        bag.close()
    return total
# Using the function to calculate the sum
bag_name = 'example_write.bag'
topic_name = 'numbers'
sum_result = calculate_sum_from_bag(bag_name, topic_name)
