import math as m
import numpy as np
import random
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString, Point, mapping
from scipy.interpolate import splprep, splev

DEBUG_PLOT = False

SCAN_WIDTH = 5

victims = []


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


def find_centroid(v):
    # find centroid from list of vertices
    # print(v)
    x, y = zip(*v)
    length = len(x)
    return [sum(x) / length, sum(y) / length]


def reorder_vertices_ccw(v):
    # lists points in CCW order based on centroid of vertices
    ctr = find_centroid(v)
    re_points = sorted(v, key=lambda p: m.atan2(p[1] - ctr[1], p[0] - ctr[0]))

    return re_points


def draw_polygon(v, alpha=1):
    # draws polygon on map from vertices (pre-sorted)
    plt.fill(list(x[0] for x in v), list(y[1] for y in v), alpha=alpha)


def get_nearest_neighbor_ind(traj, point):
    dlist = [(loc[0] - point[0]) ** 2 + (loc[1] - point[1]) ** 2 for loc in traj]
    minind = dlist.index(min(dlist))

    return minind


def remove_duplicates(traj):
    prev_val = None
    out = []

    for point in traj:
        if point != prev_val:
            out.append(point)
            prev_val = point

    return out


def generate_expanding_search(bounds, start_coord=None, step_size=1):
    if start_coord is None:
        x, y = 0, 0
    else:
        x = start_coord[0]
        y = start_coord[1]

    traj = []
    keep_expanding = True

    while keep_expanding:
        i = len(traj)
        if i % 2:  # even
            if int(i / 2) % 2:
                x += step_size * int(i / 2)
            else:
                x -= step_size * int(i / 2)
        else:  # odd
            if int(i / 2) % 2:
                y += step_size * int(i / 2)
            else:
                y -= step_size * int(i / 2)

        traj.append([x, y])

        # TODO: develop check of when to stop. check bounds on each side compared to trajectory
        # for now, stop after a while

        if len(traj) > 100:
            keep_expanding = False

    return traj


def trim_traj_to_poly(traj, poly):

    trim_traj = []

    for i in range(len(traj)-1):
        p = traj[i]
        p_ = traj[i+1]

        if poly.contains(Point(p[0], p[1])):
            trim_traj.append(p)
            if poly.contains(Point(p_[0], p_[1])):
                continue

        line = LineString([p, p_])

        if not line.intersects(poly):
            continue

        res = line.intersection(poly)

        if not res.is_empty:
            if res.geom_type == 'MultiLineString':
                continue # TODO: handle multiple linestrings. choose first?

            res = res.coords.xy
            int1 = [res[0][0], res[1][0]]
            int2 = [res[0][1], res[1][1]]

            if int1 != p and int1 != p_:
                trim_traj.append(int1)
                # if DEBUG_PLOT:
                #     plt.plot(int1[0], int1[1], 'r*')

            if int2 != p and int2 != p_:
                trim_traj.append(int2)
                # if DEBUG_PLOT:
                #     plt.plot(int2[0], int2[1], 'r*')

    trim_traj = remove_duplicates(trim_traj)

    return trim_traj


def get_full_coverage_trajectory(locs, z):
    locs = reorder_vertices_ccw(locs)

    poly_bounds = Polygon(locs)
    inflate_bounds = poly_bounds.buffer(SCAN_WIDTH)

    inflate_vertices = mapping(inflate_bounds)['coordinates']
    inflate_vertices = [list(x) for x in inflate_vertices[0]]

    if DEBUG_PLOT:
        draw_polygon(inflate_vertices)
        draw_polygon(locs)

    search_traj = generate_expanding_search(inflate_vertices, start_coord=find_centroid(inflate_vertices), step_size=SCAN_WIDTH)
    search_traj = trim_traj_to_poly(search_traj, inflate_bounds)

    x = np.array([p[0] for p in search_traj])
    y = np.array([p[1] for p in search_traj])

    # if DEBUG_PLOT:
    #     plt.plot(x, y, 'r')

    tck, u = splprep([x, y], s=1)
    unew = np.arange(0, 1.00, 0.0001)
    smooth_search_traj = splev(unew, tck)

    traj_heading = []
    for i in range(len(smooth_search_traj[0]) - 1):
        heading = np.arctan2(smooth_search_traj[1][i + 1] - smooth_search_traj[1][i], smooth_search_traj[0][i + 1] - smooth_search_traj[0][i])
        traj_heading.append(heading)

    traj_heading.append(traj_heading[-1])

    smooth_search_traj.append(z * np.ones(len(traj_heading)))
    smooth_search_traj.append(np.array(traj_heading))

    return smooth_search_traj


if __name__ == '__main__':
    # random.seed(500)
    DEBUG_PLOT = True

    MAP_BOUNDS = [[-250, 250], [-250, 250]]
    NUM_VICTIMS = 6

    if DEBUG_PLOT:
        plt.figure()

    for i in range(NUM_VICTIMS):
        rand_loc_x = random.uniform(MAP_BOUNDS[0][0], MAP_BOUNDS[0][1])
        rand_loc_y = random.uniform(MAP_BOUNDS[1][0], MAP_BOUNDS[1][1])
        new_victim = Victim(i, [rand_loc_x, rand_loc_y])
        victims.append(new_victim)

        if DEBUG_PLOT:
            plt.plot(new_victim.x, new_victim.y, 'rx')

    victim_locs = [[v.x, v.y] for v in victims]

    smooth_search_traj = get_full_coverage_trajectory(victim_locs, 5)

    if DEBUG_PLOT:
        plt.plot(smooth_search_traj[0], smooth_search_traj[1], 'k', lw=3)
        plt.axis('equal')
        plt.show()
