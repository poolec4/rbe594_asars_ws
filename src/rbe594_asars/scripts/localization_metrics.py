#!/usr/bin/env python

import rospy
import pickle
import matplotlib.pyplot as plt
import os.path
from timeit import default_timer as timer
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rbe594_asars.srv import VictimsLoc, VictimsLocResponse
import numpy as np
import tf

HOME_DIR = os.path.expanduser('~')
DEFAULT_LOCALIZATION_METRICS_FILE = os.path.join(HOME_DIR, '.ros/localization_metrics.pickle')

wheelOdom   = {'x':[], 'y':[], 'yaw': []}
imu         = {'x':[], 'y':[], 'yaw': []}
gps         = {'x':[], 'y':[], 'yaw': []}
ekf         = {'x':[], 'y':[], 'yaw': []}
gt          = {'x':[], 'y':[], 'yaw': []}


def quat_to_yaw(quat):
    euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return yaw

def robotPose_wheelOdom_cb(wheelOdom_data):
    pass

def robotPose_imu_cb(imu_data):
    pass

def robotPose_gps_cb(gps_data):
    pass

def robotPose_ekf_cb(ekf_data):
    ekf['x'].append(ekf_data.pose.pose.position.x)
    ekf['y'].append(ekf_data.pose.pose.position.y)
    ekf['yaw'].append(quat_to_yaw(ekf_data.pose.pose.orientation))


def robotPose_groundTruth_cb(groundTruth_data):
    gt['x'].append(groundTruth_data.pose.pose.position.x)
    gt['y'].append(groundTruth_data.pose.pose.position.y)
    gt['yaw'].append(quat_to_yaw(groundTruth_data.pose.pose.orientation))


def robotPose_erms_gt_vs_ekf():
    global gt, ekf
    
    err_x = []
    err_y = []
    err_yaw = []
    for _gt, _ekf in zip(gt['x'], ekf['x']):
        err_x.append(_gt-_ekf)
    for _gt, _ekf in zip(gt['y'], ekf['y']):
        err_y.append(_gt-_ekf)
    for _gt, _ekf in zip(gt['yaw'], ekf['yaw']):
        err_yaw.append(_gt-_ekf)
                     
    err_x = np.array(err_x)
    x_erms = np.sqrt(np.mean(err_x**2))

    err_y = np.array(err_y)
    y_erms = np.sqrt(np.mean(err_y**2))

    err_yaw = np.array(err_yaw)
    yaw_erms = np.sqrt(np.mean(err_yaw**2))

    print("ERMS: ", [x_erms, y_erms, yaw_erms])



def plot_all():
    global gt, ekf
    fig, axs = plt.subplots(3,1)
    axs[0].set_title('position_x')
    axs[0].set_xlabel('time points')
    axs[0].set_ylabel('x')

    axs[1].set_title('position_y')
    axs[1].set_xlabel('time points')
    axs[1].set_ylabel('y')

    axs[2].set_title('orientation_yaw')
    axs[2].set_xlabel('time points')
    axs[2].set_ylabel('yaw')

    axs[0].plot(gt['x'], 'k--', label='ground_truth')
    axs[0].plot(ekf['x'], '-', label='ekf')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(gt['y'], 'k--', label='ground_truth')
    axs[1].plot(ekf['y'], '-', label='ekf')
    axs[1].legend()
    axs[1].grid(True)

    axs[2].plot(gt['yaw'], 'k--', label='ground_truth')
    axs[2].plot(ekf['yaw'], '-', label='ekf')
    axs[2].legend()
    axs[2].grid(True)

    plt.show()


def goal_reached_cb(goal_data):
    # For more info, run $ rostopic info /move_base/result --> $ rosmsg show move_base_msgs/MoveBaseActionResult
    # status == 3 --> SUCCEEDED
    # status == 9 --> LOST
    # if goal_data.status.status == 3 or goal_data.status.status == 9:
    if goal_data.data == True:
        robotPose_erms_gt_vs_ekf()
        plot_all()


def intermediary_goal_reached_cb(goal_data):
    if goal_data.status.status == 3 or goal_data.status.status == 9:
        robotPose_erms_gt_vs_ekf()
        plot_all()


def setup_ros_comm():
    rospy.init_node('localization_metrics', anonymous=True)

    # rospy.Subscriber("/move_base/result", MoveBaseActionResult, intermediary_goal_reached_cb)
    rospy.Subscriber("asars_all_victims_reached", Bool, goal_reached_cb)

    rospy.Subscriber("/ground_truth/robotPose", Odometry, robotPose_groundTruth_cb)
    rospy.Subscriber("/odometry/filtered", Odometry, robotPose_ekf_cb)



if __name__ == "__main__":
    setup_ros_comm()
    # plot_all()
    # test_victims_loc()
    # test_lp()
    rospy.spin()


