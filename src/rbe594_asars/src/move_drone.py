#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

from scipy.spatial.transform import Rotation as R
import math as m

def position_callback(msg):
    global current_pos
    global current_orientation
    current_pos = msg.pose.position
    current_orientation = msg.pose.orientation


def init_move_node():
    global current_pos
    current_pos = None

    position_sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, position_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    while current_pos is None:
        rospy.sleep(0.1)
        print('Waiting for position callback to begin')

    return cmd_vel_pub


def fly_to(cmd_vel_pub, pose, postol=10.0, angtol=0.25):
    x, y, z, th = pose
    rate = rospy.Rate(10)
    twist_msg = Twist()

    # Move to the specified position
    while not rospy.is_shutdown():

        #TODO: perform rotation between UAV ref frame and earth
        quat = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        R_uav = R.from_quat(quat)

        # Calculate the distance between the current position and the target position
        dx = x - current_pos.x
        dy = y - current_pos.y
        dz = z - current_pos.z

        # Rotate into UAV frame
        dx, dy, dz = R_uav.apply([dx, dy, dz], inverse=True)
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

        # Calculate the angular offset for heading only
        _, _, zth = R_uav.as_euler('xyz')
        dzth = th - zth

        # Remap angles between -pi and pi
        while dzth < m.pi:
            dzth += 2*m.pi

        while dzth > m.pi:
            dzth -= 2*m.pi

        # Stop moving if the target position is reached

        position_achieved = distance < postol
        heading_achieved = dzth < angtol

        if position_achieved:
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            twist_msg.linear.z = 0
            twist_msg.angular.y = 0
            twist_msg.angular.z = 0

        if heading_achieved:
            twist_msg.angular.z = 0

        if position_achieved and heading_achieved:
            return True

        # Calculate the velocity to move towards the target position
        twist_msg.linear.x = dx * 0.25
        twist_msg.linear.y = dy * 0.25
        twist_msg.linear.z = dz * 0.1

        # Calculate the angular to move towards the target heading
        twist_msg.angular.z = dzth * 0.5

        # Publish the velocity command
        cmd_vel_pub.publish(twist_msg)

        # Wait for some time
        rate.sleep()

    # Stop moving
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0
    cmd_vel_pub.publish(twist_msg)


def follow_trajectory(cmd_pub, traj):
    # input trajectory in form [[x],[y],[z],[th]]

    pub = rospy.Publisher('mapping_status', String, queue_size=10)

    # first, fly straight up to z
    z = traj[2][0]
    th = traj[3][0]
    start_point = [0, 0, z, th]
    fly_to(cmd_pub, start_point, postol=1, angtol=0.1)
    print(f'Taking off to {start_point}')

    if not rospy.is_shutdown():
        pub.publish('mapping')

    # follow trajectory
    for i in range(len(traj[0])):
        x = traj[0][i]
        y = traj[1][i]
        z = traj[2][i]
        th = traj[3][i]
        cmd_point = [x, y, z, th]
        print(f'Commanding to {cmd_point}')
        fly_to(cmd_pub, cmd_point)

    # fly back to starting point and land
    takeoff_point = [0, 0, 0, 0]
    fly_to(cmd_pub, start_point, postol=1, angtol=0.1)
    fly_to(cmd_pub, takeoff_point, postol=1, angtol=0.1)
    print('Landing')

    if not rospy.is_shutdown():
        pub.publish('kill')


if __name__ == '__main__':

    cmd_pub = init_move_node() # TODO get start in function to work

    # Get the target position from the user
    # xin = float(input("Enter the x-coordinate: "))
    # yin = float(input("Enter the y-coordinate: "))
    # zin = float(input("Enter the z-coordinate: "))
    # zthin = float(input("Enter the heading: "))

    # fly_to(cmd_pub, [xin, yin, zin, zthin])

    traj = [[0, 0, 1, 0],
            [1, 1, 1, 2],
            [-1, 0, 1, 5],
            [0, 0, 1, -1]]

    follow_trajectory(cmd_pub, traj)
