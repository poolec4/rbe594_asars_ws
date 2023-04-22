#!/usr/bin/env python

import math
import time
import numpy as np
import rospy
import roslaunch

from map_traj_gen import get_full_coverage_trajectory
from move_drone import init_move_node, follow_trajectory
from victim_helpers import generate_victim_locations, spawn_victims, save_victims_loc
from pcd_to_occupancy_grid import main as generate_occ_grid
from rbe594_asars.srv import VictimsLoc, VictimsLocResponse

rospy.init_node('gcs')

MAP = 'small_city'
NUM_VICTIMS = 5

SCAN_Z = 25
SCAN_ANGLE = math.radians(90)
SCAN_OVERLAP = 0.5

# calculate scan parameters
SCAN_WIDTH = 2*math.cos(SCAN_ANGLE/2)*SCAN_Z*(1-SCAN_OVERLAP)
print(f'Computed SCAN_WIDTH = {SCAN_WIDTH}')

# randomly spawn victims
victims = generate_victim_locations(MAP, NUM_VICTIMS, 10)
victims_loc_poseArray = spawn_victims(victims)
save_victims_loc(victims_loc_poseArray)  # save victims location for AGV node to use later on

# get smooth UAV traj
victim_locs = [[v.x, v.y] for v in victims]
smooth_search_traj = get_full_coverage_trajectory(victim_locs, SCAN_Z, SCAN_WIDTH)

# # send traj to move node
rospy.wait_for_service("gazebo/get_link_state")
cmd_vel_pub = init_move_node()
follow_trajectory(cmd_vel_pub, smooth_search_traj)

rospy.spin()
