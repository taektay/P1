#!/usr/bin/env python3

import rospy
from ee106s24.msg import LaserScan

def callback(data):
    result = data.int_data + data.int_data
    rospy.loginfo(f"Received data: int_data = {data.int_data}, sum = {result}")
    rospy.loginfo(f"Timestamp: {data.header.stamp.to_sec()} seconds ")

def listener():
    rospy.init_node('listener', annonymous = True)
    rospy.loginfo("Listener node has started ....")


    rospy.Subscriber('EE106lab_topic', LaserScan, callback)
    rospy.loginfo("Subscribed to EElab_topic")
    
    rospy.spin()


if __name__ == '__main__':
    listener()