#!/usr/bin/env python

import math
import time
import numpy as np
import rospy
import roslaunch

from map_traj_gen import get_full_coverage_trajectory
from move_drone import init_move_node, follow_trajectory
from victim_helpers import generate_victim_locations, spawn_victims
from pcd_to_occupancy_grid import main as generate_occ_grid
from rbe594_asars.srv import VictimsLoc, VictimsLocResponse

MAP = 'small_city'
NUM_VICTIMS = 5

SCAN_Z = 25
SCAN_ANGLE = math.radians(90)
SCAN_OVERLAP = 0.5

SCAN_WIDTH = 2*math.cos(SCAN_ANGLE/2)*SCAN_Z*(1-SCAN_OVERLAP)
print(f'Computed SCAN_WIDTH = {SCAN_WIDTH}')

# get map bounds
# MAP_BOUNDS = [[-80, 123], [-48, 48]] # temporary hard coded map bounds
# MAP_BOUNDS = [[-50, 100], [-35, 35]] # Smaller map

# randomly spawn victims
victims = generate_victim_locations(MAP, NUM_VICTIMS, 50)
victims_loc_poseArray = spawn_victims(victims)

# get smooth UAV traj
victim_locs = [[v.x, v.y] for v in victims]
smooth_search_traj = get_full_coverage_trajectory(victim_locs, SCAN_Z, SCAN_WIDTH)

# # send traj to move node
rospy.wait_for_service("gazebo/get_link_state")
cmd_vel_pub = init_move_node()
follow_trajectory(cmd_vel_pub, smooth_search_traj)

# # get occupancy grid
# generate_occ_grid()

# service that provides info about victims loc
def handle_VictimsLoc(req):
    return VictimsLocResponse(victims_loc_poseArray)

rospy.init_node('gcs')
victims_loc = rospy.Service('VictimsLoc', VictimsLoc, handle_VictimsLoc)
print("VictimsLoc server up and running")

# perform global path plan

# perform local path plan


rospy.spin()