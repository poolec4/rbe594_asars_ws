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
lp_planner = "TEB"

global_path = {'x': [], 'y': []}
actual_path = {'x': [], 'y': []}
cmd_vel = []
test_globalPath = Path
global_plan_itr = 0

# Measure time
start = 0
end = 0
itr = 1

def odom_cb(odometry_data):
    # print('odom_cb')
    if  odometry_data.twist.twist.linear.x > 0.1 or odometry_data.twist.twist.linear.y > 0.1:
        # print("(x,y) (%.2f, %.2f)" % (odometry_data.pose.pose.position.x, odometry_data.pose.pose.position.y))
        actual_path['x'].append(odometry_data.pose.pose.position.x)
        actual_path['y'].append(odometry_data.pose.pose.position.y)


def save_data():
    new_data =  {'global_path': global_path,
                 'actual_path': actual_path,
                 'cmd_vel':     cmd_vel,
                 'time_taken':  end-start,
                 'test_globalPath': test_globalPath}
    
    data = {}
    filePath = 'metrics.pickle'
    if os.path.exists(filePath):
        with open('metrics.pickle', 'rb') as fd:
            data = pickle.load(fd)
    data[lp_planner] = new_data
    with open('metrics.pickle', 'wb') as fd:
        pickle.dump(data, fd, protocol=pickle.HIGHEST_PROTOCOL)


def plot_all():
    # load data
    with open('metrics.pickle', 'rb') as fd:
        data = pickle.load(fd)

    # plot path in 2D
    plt.figure('Global Path Following')
    plt.plot(data[lp_planner]['global_path']['x'], data[lp_planner]['global_path']['y'], 'k--', label='global_path')
    for k in data.keys():
        plt.plot(data[k]['actual_path']['x'], data[k]['actual_path']['y'], '-', label=k)
        plt.legend()


    plt.figure('Control Inputs')
    for k in data.keys():
        plt.plot(data[k]['cmd_vel'], '-', label=k)
        plt.legend()

    plt.figure()
    time_taken = []
    for k in data.keys():
        time_taken.append(data[k]['time_taken'])
    plt.bar(list(data.keys()), time_taken, color='blue', width=0.1)
    plt.legend()

    plt.show()


def goal_reached_cb(goal_data):
    # For more info, run $ rostopic info /move_base/result --> $ rosmsg show move_base_msgs/MoveBaseActionResult
    # status == 3 --> SUCCEEDED
    # status == 9 --> LOST
    if goal_data.status.status == 3 or goal_data.status.status == 9:
        global end
        end = timer()
        save_data()
        plot_all()
        actual_path['x'].clear()
        actual_path['y'].clear()
        global_path['x'].clear()
        global_path['y'].clear()
        cmd_vel.clear()
        itr = 1

def global_plan_cb(globaPlan_data):
    global test_globalPath, global_plan_itr
    
    if global_plan_itr == 0:    
        test_globalPath = globaPlan_data    
        for waypoint in globaPlan_data.poses:
            # print("(x,y) (%.2f, %.2f)" % (waypoint.pose.position.x, waypoint.pose.position.y))
            global_path['x'].append(waypoint.pose.position.x)
            global_path['y'].append(waypoint.pose.position.y)

    global_plan_itr = 1

def local_plan_cb(localPlan_data):
    # for waypoint in localPlan_data.poses:
    # print("(x,y) (%.2f, %.2f)" % (localPlan_data.poses[0].pose.position.x, localPlan_data.poses[0].pose.position.y))
    if itr == 1:
        global start
        start = timer()
    actual_path['x'].append(localPlan_data.poses[0].pose.position.x)
    actual_path['y'].append(localPlan_data.poses[0].pose.position.y)


def cmd_vel_cb(cmd_vel_data):
    cmd_vel.append(cmd_vel_data.linear.x)


def setup_ros_comm():
    rospy.init_node('lp_metrics', anonymous=True)

    # rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odom_cb)
    # rospy.Subscriber("/goal_reached", Bool, goal_reached_cb)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, goal_reached_cb)
    if lp_planner == 'DWA':
        rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, local_plan_cb)
    else:
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, local_plan_cb)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, global_plan_cb)

    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, cmd_vel_cb)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

def test_victims_loc():
    rospy.wait_for_service('VictimsLoc')
    try:
        srv_VictimsLoc = rospy.ServiceProxy('VictimsLoc', VictimsLoc)
        resp1 = srv_VictimsLoc()
        for id, loc in enumerate(resp1.victims_pose.poses):
            print(id, ": \n", loc)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


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


