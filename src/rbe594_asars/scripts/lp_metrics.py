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

HOME_DIR = os.path.expanduser('~')
DEFAULT_LP_METRICS_FILE = os.path.join(HOME_DIR, '.ros/lp_metrics.pickle')

lp_planner = ""
global_path = {'x': [], 'y': []}
actual_path = {'x': [], 'y': []}
cmd_vel = []
global_plan_itr = 0

# Measure time
start = 0
end = 0
itr = 1

def gp_tracking_erms():
    err_x = []
    err_y = []
    for gp, lp in zip(global_path['x'], actual_path['x']):
        err_x.append(gp-lp)
    for gp, lp in zip(global_path['y'], actual_path['y']):
        err_y.append(gp-lp)

    err_x = np.array(err_x)
    x_erms = np.sqrt(np.mean(err_x**2))

    err_y = np.array(err_y)
    y_erms = np.sqrt(np.mean(err_y**2))

    return [x_erms, y_erms]


def save_data():
    global start, end
    new_data =  {'global_path': global_path,
                 'actual_path': actual_path,
                 'cmd_vel':     cmd_vel,
                 'time_taken':  end-start,
                 'rms': gp_tracking_erms()}

    data = {}
    if os.path.exists(DEFAULT_LP_METRICS_FILE):
        with open(DEFAULT_LP_METRICS_FILE, 'rb') as fd:
            data = pickle.load(fd)
    data[lp_planner] = new_data
    with open(DEFAULT_LP_METRICS_FILE, 'wb') as fd:
        pickle.dump(data, fd, protocol=pickle.HIGHEST_PROTOCOL)


def plot_all():
    # load data
    with open(DEFAULT_LP_METRICS_FILE, 'rb') as fd:
        data = pickle.load(fd)

    # plot path in 2D
    fig, axs = plt.subplots(1,2)
    if 'DWA' in data:
        axs[0].set_title('DWA')
        axs[0].set_xlabel('x')
        axs[0].set_ylabel('y')
        axs[0].plot(data['DWA']['global_path']['x'], data['DWA']['global_path']['y'], 'k--', label='global_path')
        axs[0].plot(data['DWA']['actual_path']['x'], data['DWA']['actual_path']['y'], '-', label='DWA')
        axs[0].set_title('DWA')
        axs[0].set_xlabel('x')
        axs[0].set_ylabel('y')
        axs[0].legend()

    if 'TEB' in data:
        axs[1].set_title('TEB')
        axs[1].set_xlabel('x')
        axs[1].set_ylabel('y')
        axs[1].plot(data['TEB']['global_path']['x'], data['TEB']['global_path']['y'], 'k--', label='global_path')
        axs[1].plot(data['TEB']['actual_path']['x'], data['TEB']['actual_path']['y'], '-', label='TEB')
        axs[1].set_title('TEB')
        axs[1].set_xlabel('x')
        axs[1].set_ylabel('y')
        axs[1].legend()

    # i = 0
    # for k in data.keys():
    #     axs[i].plot(data[lp_planner]['global_path']['x'], data[lp_planner]['global_path']['y'], 'k--', label='global_path')
    #     axs[i].plot(data[k]['actual_path']['x'], data[k]['actual_path']['y'], '-', label=k)
    #     axs[i].set_title(k)
    #     axs[i].set_xlabel('x')
    #     axs[i].set_ylabel('y')
    #     axs[i].legend()
    #     i = i+1


    plt.figure('Control Inputs')
    for k in data.keys():
        plt.plot(data[k]['cmd_vel'], '-', label=k)
        plt.xlabel('time points')
        plt.ylabel('linear velocity')
        plt.ylim(bottom=0, top=1.0)
        plt.legend()
        

    plt.figure('AGV Execution Time')
    time_taken = []
    time_taken_labels = []
    for k in data.keys():
        time_taken.append(data[k]['time_taken'])
        time_taken_labels.append(k)
    plt.bar(time_taken_labels, time_taken, color='blue', width=0.1)
    plt.xlabel('Local Planner')
    plt.ylabel('time (sec)')
    plt.legend()

    # print erms value
    print('===ERMS===')
    for k in data.keys():
        if 'rms' in data[k]:
            print(k, '\n', data[k]['rms'])

    plt.show()


def goal_reached_cb(goal_data):
    if goal_data.data == True:
        global end
        end = rospy.get_time()
        save_data()
        plot_all()
        actual_path['x'].clear()
        actual_path['y'].clear()
        global_path['x'].clear()
        global_path['y'].clear()
        cmd_vel.clear()


def intermediary_goal_reached_cb(goal_data):
    # For more info, run $ rostopic info /move_base/result --> $ rosmsg show move_base_msgs/MoveBaseActionResult
    # status == 3 --> SUCCEEDED
    # status == 9 --> LOST
    if goal_data.status.status == 3 or goal_data.status.status == 9:
        global end
        end = rospy.get_time()
        save_data()
        plot_all()
        actual_path['x'].clear()
        actual_path['y'].clear()
        global_path['x'].clear()
        global_path['y'].clear()
        cmd_vel.clear()
        global itr
        itr = 1


def global_plan_cb(globaPlan_data):
    # global global_plan_itr
    
    # if global_plan_itr == 0:    
    # for waypoint in globaPlan_data.poses:
        # print("(x,y) (%.2f, %.2f)" % (waypoint.pose.position.x, waypoint.pose.position.y))
    global_path['x'].append(globaPlan_data.poses[0].pose.position.x)
    global_path['y'].append(globaPlan_data.poses[0].pose.position.y)

    # global_plan_itr = 1


def local_plan_cb(localPlan_data):
    # for waypoint in localPlan_data.poses:
    # print("(x,y) (%.2f, %.2f)" % (localPlan_data.poses[0].pose.position.x, localPlan_data.poses[0].pose.position.y))
    global itr
    if itr == 1:
        global start
        start = rospy.get_time()
    actual_path['x'].append(localPlan_data.poses[0].pose.position.x)
    actual_path['y'].append(localPlan_data.poses[0].pose.position.y)
    itr = itr + 1

def cmd_vel_cb(cmd_vel_data):
    cmd_vel.append(cmd_vel_data.linear.x)


def setup_ros_comm():
    rospy.init_node('lp_metrics', anonymous=True)

    # rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odom_cb)
    # rospy.Subscriber("/goal_reached", Bool, goal_reached_cb)
    # rospy.Subscriber("/move_base/result", MoveBaseActionResult, intermediary_goal_reached_cb)
    rospy.Subscriber("asars_all_victims_reached", Bool, goal_reached_cb)
    

    global lp_planner
    param_lp = rospy.get_param('/move_base/base_local_planner')
    if 'Teb' in param_lp:
        lp_planner = 'TEB'
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, local_plan_cb)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, global_plan_cb)
    elif 'DWA' in param_lp:
        lp_planner = 'DWA'
        rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, local_plan_cb)
        rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, global_plan_cb)
    else:
        lp_planner = 'ERR'
        
    # rospy.Subscriber("/move_base/NavfnROS/plan", Path, global_plan_cb)
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, cmd_vel_cb)



def test_lp():
    # rospy.init_node('test_lp')    # cuz setup_comm() is already creating a node
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 30.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Command executed!")
    


if __name__ == "__main__":
    setup_ros_comm()
    # plot_all()
    # test_victims_loc()
    # test_lp()
    rospy.spin()


