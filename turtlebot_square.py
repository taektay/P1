import rospy
from geometry_msgs.msg import Twist

rospy.init_node('turtlebot_square', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def move_forward(distance):
    vel_msg = Twist()
    vel_msg.linear.x = 0.2  # Set forward speed
    distance_moved = 0.0

    t0 = rospy.Time.now().to_sec()

    while distance_moved < distance:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        distance_moved = 0.2 * (t1 - t0)  # Update distance moved

    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)

def rotate():
    vel_msg = Twist()
    vel_msg.angular.z = 0.5  # Set rotation speed
    angle_moved = 0.0

    t0 = rospy.Time.now().to_sec()

    while angle_moved < 1.5708:  # 90 degrees in radians
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        angle_moved = 0.5 * (t1 - t0)  # Update angle moved

    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)
  
def main():
    rospy.init_node('turtlebot_square', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    for _ in range(4):
        move_forward(4)  # Move forward 4 meters
        rotate()  # Rotate 90 degrees

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
