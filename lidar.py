#!/usr/bin/env python3

import rospy
from ee106s24.msg import LaserScan
from std_msgs.msg import String

def process_scan_data(ranges, pub):
    # Process the list of ranges captured by LIDAR
    min_distance = float('inf') # Initialize with a large number

    # Check
    status = "minor"
    for distance in ranges:
        if distance >0:
            if distance < 0.2:
                status = "critical"
                break
            elif distance < 0.5:
                status = "major"
    
    # Publish the status
    status_msg = String()
    status_msg.data = status
    pub.publish(status_msg)

    rospy.loginfo("Published status: {}".format(status))

def lidar_callback(msg, pub):
    # This function gets called every time a new LaserScan message is received
    rospy.loginfo("Received a scan with %d measurements." % len(msg.ranges))

    process_scan_data(msg.ranges, pub)

def listener():
    rospy.init_node('lidar_listener', anonymous = True)
    
    # Setting up Publisher
    pub = rospy.Publisher('jackal_robot_status', String, queue_size = 10)

    # Subscribe to the LIDAR Topic
    rospy.Subscriber('/scan', LaserScan, lidar_callback, callback_args=(pub))

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()
