#!/usr/bin/env python

import time
import numpy as np
import random

from map_traj_gen import get_full_coverage_trajectory, Victim
from move_drone import init_move_node, follow_trajectory

NUM_VICTIMS = 5
SCAN_Z = 10
SCAN_WIDTH = 10

victims = []

# get map bounds
# TODO: pull from gazebo model
MAP_BOUNDS = [[-200, 200], [-200, 200]] # temporary hard coded map bounds

# randomly spawn victims
for i in range(NUM_VICTIMS):
    rand_loc_x = random.uniform(MAP_BOUNDS[0][0], MAP_BOUNDS[0][1])
    rand_loc_y = random.uniform(MAP_BOUNDS[1][0], MAP_BOUNDS[1][1])
    new_victim = Victim(i, [rand_loc_x, rand_loc_y])
    victims.append(new_victim)

# get smooth UAV traj

victim_locs = [[v.x, v.y] for v in victims]
smooth_search_traj = get_full_coverage_trajectory(victim_locs, SCAN_Z, SCAN_WIDTH)

# send traj to move node
cmd_vel_pub = init_move_node()
follow_trajectory(cmd_vel_pub, smooth_search_traj)

# get point cloud

# pass lidar map to occupancy gen
