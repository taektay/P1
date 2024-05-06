#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from ee106s24.msg import EE106lab_custom_new
import random

def talker():
    rospy.init_node('talker')
    pub = rospy.Publisher('EE106lab_topic', EE106lab_custom_new, queue_size = 10)
    rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    custom_msg = EE106lab_custom_new()
    custom_msg.header = Header()
    custom_msg.header.stamp = rospy.Time.now()
    custom_msg.int_data = random.randint(-100, 100)
    custom_msg.float_data = random.random() * 100
    custom_msg.string_data = "Random value messsage"

    rospy.loginfo(f"Publishing: int_data = {custom_msg.int_data}, float_data = {custom_msg.float_data}")
    pub.Publisher(custom_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass