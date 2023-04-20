#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *

import rospkg
import random
import numpy as np

class Victim:
    def __init__(self, id, location, severity=None):
        self.id = id
        self.x = location[0]
        self.y = location[1]

        if len(location) == 3:
            self.z = location[2]
        else:
            self.z = None

        self.severity = severity

    def __str__(self):
        print_str = f'Location: [{self.x:.2f}, {self.y:.2f}'

        if self.z is not None:
            print_str += f', {self.z:.2f}]'
        else:
            print_str += ']'

        return print_str


def generate_equidistant_points(p1, p2, n):
    return list(zip(np.linspace(p1[0], p2[0], n),
               np.linspace(p1[1], p2[1], n)))


def generate_victim_locations(map, num_victims, seed=None):
    victims = []

    if seed is not None:
        random.seed(seed)

    if map == 'small_city':
        # MAP_BOUNDS = [[-80, 123], [-48, 48]]
        perimeter_offset = 5
        min_x = -50 + perimeter_offset
        max_x = 123 - perimeter_offset
        min_y = -48 + perimeter_offset
        max_y = 48 - perimeter_offset

        victim_options = []
        victim_options.extend(generate_equidistant_points([min_x, min_y], [min_x, max_y], 5))
        victim_options.extend(generate_equidistant_points([min_x, min_y], [max_x, min_y], 5))
        victim_options.extend(generate_equidistant_points([max_x, min_y], [max_x, max_y], 5))
        victim_options.extend(generate_equidistant_points([min_x, max_y], [max_x, max_y], 5))
    else:
        victim_options = [[0, 0]]

    print(victim_options)
    rand_inds = random.sample(range(len(victim_options)), num_victims)

    for ind in rand_inds:
        rand_loc_x = victim_options[ind][0]
        rand_loc_y = victim_options[ind][1]
        new_victim = Victim(len(victims), [rand_loc_x, rand_loc_y])
        victims.append(new_victim)
        print(f'generated victim {new_victim}')

    return victims

def spawn_victims(victims):
    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('rbe594_asars')
    MODEL_PATH = pkg_path + '/models/human_male_1/model.sdf'

    print(f'opening: {MODEL_PATH}')

    with open(MODEL_PATH, "r") as f:
        model_xml = f.read()

    victims_loc = PoseArray()
    for victim in victims:
        item_name = f'Victim {victim.id}'

        orient = Quaternion(0, 0, 0, 1)
        victim_pose = Pose(Point(x=victim.x, y=victim.y, z=0), orient)
        victims_loc.poses.append(victim_pose)

        spawn_model(item_name, model_xml, '', victim_pose, "world")
        print(f'spawned {item_name}')

    return victims_loc
