#!/usr/bin/env python


# This script generates a costmap from a point cloud data (PCD) file, taking into account the slope angles
# between adjacent cells in the costmap. The costmap is then published as an OccupancyGrid message in ROS.
# If the slope value exceed the angle threshold, it assigns a 100 cost. Cost assignment for angle between 0 to THRESHOLD_VALUE
# increases as slope getting closer to threshold value.
# Fill Gaps method is used to adress the issue of any missing or unassigned cost in the costmap due to the nature of point cloud data.


import pcl
import rospy
import rospkg

import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Quaternion
import math

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('rbe594_asars')

DEFAULT_PCD_FILE = pkg_path + '/map_grid.pcd'
DEFAULT_RESOLUTION = 0.5
DEFAULT_ANGLE_THRESHOLD = 30

def load_pcd_file(filename):
    try:
        cloud = pcl.load(filename)
        return cloud.to_list()
    except Exception as e:
        rospy.logerr(f"Failed to load PCD file: {filename}. Error: {e}")
        return []
    

def create_costmap_from_point_cloud(points, resolution=0.5, max_angle=30):
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)

    size_x = int(np.ceil((max_coords[0] - min_coords[0]) / resolution) + 1)
    size_y = int(np.ceil((max_coords[1] - min_coords[1]) / resolution) + 1)

    height_grid = np.full((size_y, size_x), np.nan)
    count_grid = np.zeros((size_y, size_x), dtype=int)

    for x, y, z in points:
        i = int((y - min_coords[1]) / resolution)
        j = int((x - min_coords[0]) / resolution)

        if np.isnan(height_grid[i, j]):
            height_grid[i, j] = z
        else:
            height_grid[i, j] += z
        count_grid[i, j] += 1

    height_grid /= np.maximum(count_grid, 1)

    # Create the costmap from the height grid
    costmap = np.full((size_y, size_x), -1, dtype=np.int8)
    max_cost = 100
    angle_threshold = np.deg2rad(max_angle)

    for i in range(size_y):
        for j in range(size_x):
            if np.isnan(height_grid[i, j]):
                continue

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                ni, nj = i + dy, j + dx
                if 0 <= ni < size_y and 0 <= nj < size_x and not np.isnan(height_grid[ni, nj]):
                    dz = height_grid[i, j] - height_grid[ni, nj]
                    distance = resolution * np.sqrt(dx*dx + dy*dy)
                    slope = np.arctan2(dz, distance)

                    if slope > angle_threshold:  # Exceeding max_angle is treated as an obstacle
                        cost = max_cost
                    else:
                        cost = max(0, int(max_cost * (slope / angle_threshold)))

                    # Clip the cost values before updating the costmap
                    clipped_cost = np.clip(cost, -128, 127)
                    costmap[i, j] = max(costmap[i, j], clipped_cost)

     # Fill gaps in the costmap
    for i in range(1, size_y - 1):
        for j in range(1, size_x - 1):
            if costmap[i, j] == -1:
                neighbors = [
                    costmap[i - 1, j],
                    costmap[i + 1, j],
                    costmap[i, j - 1],
                    costmap[i, j + 1],
                ]
                known_neighbors = [value for value in neighbors if value != -1]

                if len(known_neighbors) >= len(neighbors) - 1:
                    costmap[i, j] = int(sum(known_neighbors) / len(known_neighbors))

    return costmap


def create_costmap_message(costmap, resolution, min_coords):
    costmap_msg = OccupancyGrid()
    costmap_msg.header.frame_id = "map"
    costmap_msg.header.stamp = rospy.Time.now()
    costmap_msg.info.resolution = resolution
    costmap_msg.info.width = costmap.shape[1]
    costmap_msg.info.height = costmap.shape[0]
    costmap_msg.info.origin.position.x = min_coords[0]
    costmap_msg.info.origin.position.y = min_coords[1]

    clipped_costmap = np.clip(costmap, -128, 127)
    costmap_msg.data = (costmap.flatten()).astype(np.int8).tolist()

    return costmap_msg


def main():
    rospy.init_node('costmap_from_pcd')

    pcd_filename = rospy.get_param("~pcd_file", DEFAULT_PCD_FILE)

    point_cloud = load_pcd_file(pcd_filename)

    resolution = rospy.get_param("~resolution", DEFAULT_RESOLUTION)
    max_angle = rospy.get_param("~max_angle", DEFAULT_ANGLE_THRESHOLD)

    min_coords = np.min(point_cloud, axis=0)
    costmap_np = create_costmap_from_point_cloud(point_cloud, resolution=resolution, max_angle=max_angle)

    costmap_msg = create_costmap_message(costmap_np, resolution, min_coords)

    costmap_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        costmap_pub.publish(costmap_msg)
        rate.sleep()

if __name__ == '__main__':
    main()

