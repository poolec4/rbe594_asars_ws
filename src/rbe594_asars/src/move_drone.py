#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped

def position_callback(msg):
    global current_pos
    current_pos = msg.pose.position

def fly_to(x, y, z):
    twist_msg = Twist()

    # Move to the specified position
    while not rospy.is_shutdown():
        # Calculate the distance between the current position and the target position
        dx = x - current_pos.x
        dy = y - current_pos.y
        dz = z - current_pos.z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

        # Stop moving if the target position is reached
        if distance < 0.5:
            break

        # Calculate the velocity to move towards the target position
        twist_msg.linear.x = dx * 0.2
        twist_msg.linear.y = dy * 0.2
        twist_msg.linear.z = dz * 0.2

        # Publish the velocity command
        cmd_vel_pub.publish(twist_msg)

        # Wait for some time
        rate.sleep()

    # Stop moving
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    twist_msg.linear.z = 0
    cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('fly_to_position')
    rate = rospy.Rate(10)

    current_pos = None
    position_sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, position_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    while not current_pos:
        rospy.sleep(0.1)

    # Get the target position from the user
    x = float(input("Enter the x-coordinate: "))
    y = float(input("Enter the y-coordinate: "))
    z = float(input("Enter the z-coordinate: "))

    fly_to(x, y, z)

