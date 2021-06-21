#!/usr/bin/env python
"""
Probabilistic Road Map (PRM) Planner
author: Atsushi Sakai (@Atsushi_twi)
"""

import random
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from utils_hb import *

from detection import *

from look_object import *

# parameter
N_SAMPLE = 3000 # number of sample_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

show_animation = True


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) + "," + str(self.parent_index)


def prm_planning(sx, sy, gx, gy, ox, oy, rr):

    obstacle_kd_tree = cKDTree(np.vstack((ox, oy)).T)

    sample_x, sample_y = sample_points(sx, sy, gx, gy,
                                       rr, ox, oy, obstacle_kd_tree)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree)

    rx, ry = dijkstra_planning(
        sx, sy, gx, gy, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    n_step = round(d / D)

    for i in range(int(n_step)):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    dist, _ = obstacle_kd_tree.query([gx, gy])
    if dist <= rr:
        return True  # collision

    return False  # OK


def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
    """
    Road map generation
    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obstacle_kd_tree: KDTree object of obstacles
    """

    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = cKDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
    """
    s_x: start x position [m]
    s_y: start y position [m]
    gx: goal x position [m]
    gy: goal y position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    rr: robot radius [m]
    road_map: ??? [m]
    sample_x: ??? [m]
    sample_y: ??? [m]
    @return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
    """

    start_node = Node(sx, sy, 0.0, -1)
    goal_node = Node(gx, gy, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True

    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        # show graph
        # comment for eliminate the grid
        #if show_animation and len(closed_set.keys()) % 2 == 0:
        #    # for stopping simulation with the esc key.
        #    plt.gcf().canvas.mpl_connect(
        #        'key_release_event',
        #        lambda event: [exit(0) if event.key == 'escape' else None])
        #    plt.plot(current.x, current.y, "xg")
        #    plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove the item from the open set
        del open_set[c_id]
        # Add it to the closed set
        closed_set[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closed_set:
                continue
            # Otherwise if it is already in the open set
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

    # generate final course
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


# samples points
def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree):
    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)

    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        for i in range(min_x, max_x):
            tx = (i * (max_x - min_x)) + min_x
        tx = (random.random() * (max_x - min_x)) + min_x
        ty = (random.random() * (max_y - min_y)) + min_y

        #for i in range(20):
        #    for j in range(40):
        #        print(j)
        #        tx = (i)
        #        ty = (j)


        dist, index = obstacle_kd_tree.query([tx, ty])

        if dist >= rr:
            sample_x.append(tx)
            sample_y.append(ty)
    #sample_x.append(16)
    #sample_y.append(41)
    #sample_x.append(5)
    #sample_y.append(54)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    #print(sample_x)
    #print(sample_y)

    return sample_x, sample_y


def jo(x_1, y_1, x_2, y_2):
    print(__file__ + " start!!")

    # start and goal position
    sx = 27.2  # [m]
    sy = 54.6  # [m]
    #sx = 26.5
    #sy = 20.1
    gx = 41.5 # [m]
    gy = 51.2  # [m]
    #gx = 45.0 # [m]
    #gy = 45.0  # [m]
    robot_size = 2.5 # [m]

    ox = []
    oy = []

    # A-B
    for i in range(0, 60):
        ox.append(i)
        oy.append(40)
    # B-C
    for i in range(40, 61):
        ox.append(60)
        oy.append(i)
    # D-C
    for i in range(0, 60):
        ox.append(i)
        oy.append(60)
    # A-D
    for i in range(40, 60):
        ox.append(0)
        oy.append(i)
    # Wall
    for i in range(40, 50):
        ox.append(27)
        oy.append(i)
    for i in range(40, 50):
        ox.append(30)
        oy.append(i)
    for i in range(28, 30):
        ox.append(i)
        oy.append(50)

    # obstacules
    ox.append(x_1)
    oy.append(y_1)


    ox.append(x_2)
    oy.append(y_2)

    #if show_animation:
    #    plt.plot(ox, oy, ".k")
    #    plt.plot(sx, sy, "^r")
    #    plt.plot(gx, gy, "^c")
    #    plt.grid(True)
    #    plt.axis("equal")

    rx, ry = prm_planning(sx, sy, gx, gy, ox, oy, robot_size)

    assert rx, 'Cannot found path'


    #if show_animation:
    #    plt.plot(rx, ry, "-r")
    #    print(rx, len(rx), ry, len(ry), type(ry), type(rx))
    #    plt.pause(0.001)
    #    plt.show()
    print(rx, len(rx), ry, len(ry), type(ry), type(rx))

    for i in range(len(rx)):
        rx[i] = (rx[i]-7.1)/10

    for i in range(len(ry)):
        ry[i] = (ry[i]-28.1)/10

    #print(" ")

    #print(rx, ry)

    return rx, ry

#if __name__ == '__main__':
#    main()
#    print(len(rx), len(ry))

A = jo(31.2, 54.4, 39, 57)
#print(" ")
#print(" ")
#print(A[1])
#print(" ")
#print(" ")
#print(A[0])

if __name__ == '__main__':
    rospy.init_node('fish')
    n = len(A[0])
    x = []
    y = []
    for item in reversed(A[0]):
        y.append(item)
    for item in reversed(A[1]):
        x.append(item)

    A = RGBD()
    pose = A.detect()

    poses = A.detect()[0][0]
    n = len(poses)
    print(poses, "  ", n)

    x_obs = []
    y_obs = []
    z_obs = []

    #for i in range(n):
    #    x_obs.append(poses[i][0]*10)
    #    y_obs.append(poses[i][1]*10)
    #    z_obs.append(poses[i][2]*10)

    real_pose = utils_hb.make_pose_from_camera(poses)
    map_pose = utils_hb.get_pose_relative_coordinate('map', real_pose)
    utils_hb.rviz_marker('map', map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z)
    print(map_pose)

    #for i in range(n):
    #    #main()
    #    move_head_tilt(-1)
    #    m = Move()
    #    m.set_pose(x[i], y[i], 130)
    #    #m.get_pose()
    #    m.go()
