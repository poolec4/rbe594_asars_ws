#!/usr/bin/env python

import pcl
import numpy as np

DEFAULT_RESOLUTION = 0.5


def load_pcd_file(filename):
    print("Loading PCD file")
    try:
        cloud = pcl.load(filename)
        return cloud.to_list()
    except Exception as e:
        print("Failed to load pcd file")
        return []


def point_cloud_to_height_grid(point_cloud, resolution):
    print("Converting PCD to height grid")
    if not point_cloud:
        # rospy.logwarn("Point cloud is empty.")
        print("Point cloud is empty")
        return np.zeros((0, 0), dtype=np.uint8), np.array([0, 0, 0])
    
    min_coords = np.min(point_cloud, axis=0)
    max_coords = np.max(point_cloud, axis=0)


    width = int(np.ceil((max_coords[0] - min_coords[0]) / resolution) + 1)
    height = int(np.ceil((max_coords[1] - min_coords[1]) / resolution) + 1) 

    grid = np.full((width, height), -1, dtype=np.int8)

    for point in point_cloud:
        x = int(np.floor((point[0] - min_coords[0]) / resolution))
        y = int(np.floor((point[1] - min_coords[1]) / resolution))
        # print(f"point: {point}, x: {x}, y: {y}")

        grid[x, y] = point[2]

    # Interpolate unknown cells between recorded cells
    for x in range(1, width - 1):
        for y in range(1, height - 1):
            if grid[x, y] == -1:
                neighbors = [
                    grid[x - 1, y],
                    grid[x + 1, y],
                    grid[x, y - 1],
                    grid[x, y + 1],
                ]
                known_neighbors = [value for value in neighbors if value != -1]

                if len(known_neighbors) >= len(neighbors) - 1:
                    value = int(sum(known_neighbors) / len(known_neighbors))
                    grid[x, y] = value

    
    origin = np.array([min_coords[0], min_coords[1] , 0])
   
    return grid, origin


def main():
    # Put the pcd files in the same directory as the script and specify the filenames in the list
    pcd_files = ["map_grid_1.pcd", "map_grid_2.pcd", "map_grid_3.pcd", "map_grid_4.pcd", "map_grid_5.pcd", 
                 "map_grid_6.pcd", "map_grid_7.pcd", "map_grid_8.pcd", "map_grid_9.pcd", "map_grid_10.pcd"]
    grids = []
    origins = []

    for k in range(len(pcd_files)):
        pcd_filename = pcd_files[k]
        point_cloud = load_pcd_file(pcd_filename)
        height_grid, origin = point_cloud_to_height_grid(point_cloud, DEFAULT_RESOLUTION)
        grids.append(height_grid)
        origins.append(origin)
    print("Loaded all PCD files")

    print("Starting MSE calculation")
    base_pcd = grids[-1]
    n = int(len(base_pcd)*len(base_pcd[0])*len(pcd_files))
    MSE = 0
    for i in range(len(pcd_files)-1):
        current_pcd = grids[i]
        for x in range(len(base_pcd)):
            for y in range(len(base_pcd[0])):
                if x >= len(current_pcd) or y >= len(current_pcd[0]):
                    continue

                base_val = base_pcd[x][y]
                current_val = current_pcd[x][y]
                MSE += (1/n)*(base_val-current_val)**2

    print("Mean Squared Error across ", len(pcd_files), "maps: ", MSE)


if __name__ == "__main__":
    main()
