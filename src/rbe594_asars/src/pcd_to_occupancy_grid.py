#!/usr/bin/env python

# This script generates a Occupancy map from a point cloud data (PCD) file, taking into account the height threshold.
# The occupancymap is then published as an OccupancyGrid message in ROS.
# If the height value exceed the height threshold, it assigns a 100(occupied), otherwise it assigns 0(unoccupied).
# Interpolate method is used to adress the issue of any missing or unassigned occupation in the occupancygrid due to the nature of point cloud data.

import rospy
import pcl
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import os
from std_srvs.srv import Empty
from victim_helpers import load_victims_loc, spawn_victims, Victim
from asars_global_planner.srv import GenerateVisitingOrder
HOME_DIR = os.path.expanduser('~')
DEFAULT_PCD_FILE = os.path.join(HOME_DIR, '.ros/map_grid.pcd')
DEFAULT_RESOLUTION = 0.5
DEFAULT_HEIGHT_THRESHOLD = 0.5


def load_pcd_file(filename):
    try:
        cloud = pcl.load(filename)
        return cloud.to_list()
    except Exception as e:
        rospy.logerr(f"Failed to load PCD file: {filename}. Error: {e}")
        return []


def point_cloud_to_occupancy_grid(point_cloud, resolution, height_threshold):
    if not point_cloud:
        rospy.logwarn("Point cloud is empty.")
        return np.zeros((0, 0), dtype=np.uint8), np.array([0, 0, 0])
    
    min_coords = np.min(point_cloud, axis=0)
    max_coords = np.max(point_cloud, axis=0)


    width = int(np.ceil((max_coords[0] - min_coords[0]) / resolution) + 1)
    height = int(np.ceil((max_coords[1] - min_coords[1]) / resolution) + 1) 

    occupancy_grid = np.full((width, height), -1, dtype=np.int8)

    for point in point_cloud:
        x = int(np.floor((point[0] - min_coords[0]) / resolution))
        y = int(np.floor((point[1] - min_coords[1]) / resolution))
        #print(f"point: {point}, x: {x}, y: {y}")

        if point[2] > height_threshold:
            occupancy_grid[x, y] = 100
        
        else:
            occupancy_grid[x, y] = 0

# Interpolate unknown cells between recorded cells
    for x in range(1, width - 1):
        for y in range(1, height - 1):
            if occupancy_grid[x, y] == -1:
                neighbors = [
                    occupancy_grid[x - 1, y],
                    occupancy_grid[x + 1, y],
                    occupancy_grid[x, y - 1],
                    occupancy_grid[x, y + 1],
                ]
                known_neighbors = [value for value in neighbors if value != -1]

                if len(known_neighbors) >= len(neighbors) - 1:
                    value = int(sum(known_neighbors) / len(known_neighbors))

                    if value > height_threshold:
                        occupancy_grid[x, y] = 100
                    else:
                        occupancy_grid[x, y] = 0

    
    origin = np.array([min_coords[0], min_coords[1] , 0])
   
    return occupancy_grid, origin


def numpy_to_occupancy_grid_msg(occupancy_grid_np, resolution, origin):
    occupancy_grid_msg = OccupancyGrid()

    occupancy_grid_msg.header.stamp = rospy.Time.now()
    occupancy_grid_msg.header.frame_id = "map"
    occupancy_grid_msg.info.resolution = resolution
    occupancy_grid_msg.info.width = occupancy_grid_np.shape[0]
    occupancy_grid_msg.info.height = occupancy_grid_np.shape[1]
    occupancy_grid_msg.info.origin = Pose(Point(origin[0], origin[1], 0), Quaternion(0, 0, 0, 1))
    occupancy_grid_msg.data = (occupancy_grid_np.T.flatten()).astype(np.int8).tolist()
    
    return occupancy_grid_msg

def start_planner_cb(req):
    print("Planner request started")
    print("waiting for victim generation service")
    rospy.wait_for_service('asars/start_victim_visting')
    victim_locations = load_victims_loc()
    try:
        generate_visiting_order_srv = rospy.ServiceProxy('asars/start_victim_visting', GenerateVisitingOrder)
        point_msg_array = [poses.position for poses in victim_locations.poses]
        victims = []
        for pose in victim_locations.poses:
            victims.append(Victim(len(victims), [pose.position.x, pose.position.y]))

        spawn_victims(victims)
        resp1 = generate_visiting_order_srv(point_msg_array)
        return resp1.victim_visiting_order
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# This service is to be called to start the global planning nodes
def start_planner_srv():
    planner_srv = rospy.Service('/asars/start_planning', Empty, start_planner_cb)
    return planner_srv


def main():
    pcd_filename = rospy.get_param("~pcd_file", DEFAULT_PCD_FILE)
    resolution = rospy.get_param("~resolution", DEFAULT_RESOLUTION)
    height_threshold = rospy.get_param("~height_threshold", DEFAULT_HEIGHT_THRESHOLD)

    point_cloud = load_pcd_file(pcd_filename)

    occupancy_grid_np, origin = point_cloud_to_occupancy_grid(point_cloud, resolution, height_threshold)
        
    occupancy_grid_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)

    rate = rospy.Rate(0.5)  # Publish at 0.5 Hz
    while not rospy.is_shutdown():
        occupancy_grid_msg = numpy_to_occupancy_grid_msg(occupancy_grid_np, resolution, origin)
        occupancy_grid_pub.publish(occupancy_grid_msg)
        rospy.loginfo("Published occupancy grid.")
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("pcd_to_occupancy_grid")
    
    # start occupancy grid
    start_planner_srv = start_planner_srv()
    main()
