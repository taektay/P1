#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist  # Carry linear and angular velocity
from math import pi, cos, sin  # for sin, cos, and angular velocities
import csv  # not sure if this is necessary

# Given Sample Code
class Turtlebot():
    # Initialize the node
    def __init__(self):                                                            
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        # Initialize position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Create or clear the trajectory file
        self.trajectory_file = "trajectory.csv"
        with open(self.trajectory_file, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "theta"])

        self.run()  # Self run

    # Function to log the current position
    def log_position(self):
        with open(self.trajectory_file, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([self.x, self.y, self.theta])
            
    # Function to update the robot's position based on distance and angle moved
    def update_position(self, distance, angle):
        self.theta += angle
        self.x += distance * cos(self.theta)
        self.y += distance * sin(self.theta)
        self.log_position()

    # Define Function: 'move_forward' including speed and distance
    def move_forward(self, speed, distance):
        vel = Twist()  # Linear Velocity
        vel.linear.x = abs(speed)
        vel.angular.z = 0

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        # Open Loop
        while current_distance < distance:
            self.vel_pub.publish(vel)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)
            self.rate.sleep()

        vel.linear.x = 0
        self.vel_pub.publish(vel)

        # Update position after moving forward
        self.update_position(distance, 0)

    # Define Function: 'rotate' open loop using while loop
    def rotate(self, angular_speed, relative_angle):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = abs(angular_speed)

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < relative_angle:
            self.vel_pub.publish(vel)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            self.rate.sleep()

        vel.angular.z = 0
        self.vel_pub.publish(vel)

        # Update position after rotating
        self.update_position(0, relative_angle)

    # Define Function: 'run'
    def run(self):
        # Square movement parameters
        speed = 0.2
        distance = 4
        angular_speed = 0.5
        relative_angle = pi / 2  # 90 degrees in radians

        for _ in range(4):
            self.move_forward(speed, distance)
            self.rotate(angular_speed, relative_angle)

if __name__ == '__main__':
    try:
        tb = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
