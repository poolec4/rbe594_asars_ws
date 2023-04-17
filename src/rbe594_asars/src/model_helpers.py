#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospkg


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
        # (self, id, location, severity=None):
        item_name = f'Victim {victim.id}'

        orient = Quaternion(0, 0, 0, 1)
        victim_pose = Pose(Point(x=victim.x, y=victim.y, z=0), orient)
        victims_loc.poses.append(victim_pose)

        spawn_model(item_name, model_xml, '', victim_pose, "world")
        print(f'spawned {item_name}')

    return victims_loc
