"""
https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/AStar
"""

import math

import matplotlib.pyplot as plt

import numpy as np

import time
from numpy.linalg import norm as norm
from pyrsistent import v

import globalvar
from globalvar import vclass
from globalvar import hybrid_astar_Set

from main_unstructure import CreateVehiclePolygon


from CheckByAABB import Vehicle_Allobj_AABB_Collision
from CheckByCircle import Vehicle_Allobj_Circumscribed_Circle_Collision

global astar_
astar_ = hybrid_astar_Set()


class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x  # index of grid
        self.y = y  # index of grid
        self.cost = cost
        self.parent_index = parent_index


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion

def calc_xy_index(position, dim):
    if(dim == 0):
        pos = math.ceil(position / astar_.resolution_x)
    if(dim == 1):
        pos = math.ceil(position / astar_.resolution_y)
    return pos


def calc_grid_index(node):
    return node.y * astar_.num_nodes_x + node.x


def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
    return d


def calc_grid_position(index, dim):
    """
    calc grid position

    :param index:
    :param min_position:
    :return:
    """
    if(dim == 0):
        pos = index * astar_.resolution_x
    if(dim == 1):
        pos = index * astar_.resolution_y
    return pos


def calc_final_path(goal_node, closed_set):
    # generate final course
    rx, ry = [calc_grid_position(goal_node.x, 0)], [
        calc_grid_position(goal_node.y, 0)]
    length = 0
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(calc_grid_position(n.x, 0))
        ry.append(calc_grid_position(n.y, 0))
        length = length+math.sqrt((rx[0]-rx[1])**2+(ry[0]-ry[1])**2)
        parent_index = n.parent_index

    return rx, ry, length


def verify_node(idx, idy):
    x = calc_grid_position(idx, 0)
    y = calc_grid_position(idy, 1)
    if((y - globalvar.vehicle_geometrics_.vehicle_width / 2 <= globalvar.planning_scale_.ymin) or
       (y + globalvar.vehicle_geometrics_.vehicle_width / 2 >= globalvar.planning_scale_.ymax) or
        (x - globalvar.vehicle_geometrics_.vehicle_rear_hang <= globalvar.planning_scale_.xmin) or
        (x + globalvar.vehicle_geometrics_.vehicle_front_hang +
         globalvar.vehicle_geometrics_.vehicle_wheelbase >= globalvar.planning_scale_.xmax)
       ):
        return False
    # # collision check
    # if (Vehicle_Allobj_AABB_Collision(x, y, 0, globalvar.obstacles_) == True):
    #     return False

    
    V = CreateVehiclePolygon(x, y, 0)
    idx0 = calc_xy_index(V.x[0], 0)
    idy0 = calc_xy_index(V.y[0], 1)
    idx1 = calc_xy_index(V.x[1], 0)
    idy1 = calc_xy_index(V.y[1], 1)
    idx2 = calc_xy_index(V.x[2], 0)
    idy2 = calc_xy_index(V.y[2], 1)
    idx3 = calc_xy_index(V.x[3], 0)
    idy3 = calc_xy_index(V.y[3], 1)
    minx = min(idx0, idx1, idx2, idx3)
    miny = min(idy0, idy1, idy2, idy3)
    maxx = max(idx0, idx1, idx2, idx3)
    maxy = max(idy0, idy1, idy2, idy3)
    for x in range(minx, maxx,2):
        if(map[x][miny] == 0):
            return False
    for x in range(minx, maxx,2):
        if(map[x][maxy] == 0):
            return False
    for y in range(miny, maxy,2):
        if(map[minx][y] == 0):
            return False
    for y in range(miny, maxy,2):
        if(map[maxx][y] == 0):
            return False
    return True


def search_closest(ids, goalx, goaly, startx, starty):
    xx = startx
    yy = starty
    for id in ids:
        x = id % astar_.num_nodes_x
        y = round((id-x)/astar_.num_nodes_x)
        if(math.fabs(xx-goalx)+math.fabs(yy-goaly) > math.fabs(x-goalx)+math.fabs(y-goaly)):
            xx = x
            yy = y
    return [xx, yy]


def costmap():
    map = np.ones((round(astar_.num_nodes_x), round(astar_.num_nodes_y)))
    for obj in globalvar.obstacles_[0]:
        o_idx0 = calc_xy_index(obj.x[0], 0)
        o_idy0 = calc_xy_index(obj.y[0], 1)
        o_idx1 = calc_xy_index(obj.x[1], 0)
        o_idy1 = calc_xy_index(obj.y[1], 1)
        o_idx2 = calc_xy_index(obj.x[2], 0)
        o_idy2 = calc_xy_index(obj.y[2], 1)
        o_idx3 = calc_xy_index(obj.x[3], 0)
        o_idy3 = calc_xy_index(obj.y[3], 1)
        minx = min(o_idx0, o_idx1, o_idx2, o_idx3)
        miny = min(o_idy0, o_idy1, o_idy2, o_idy3)
        maxx = max(o_idx0, o_idx1, o_idx2, o_idx3)
        maxy = max(o_idy0, o_idy1, o_idy2, o_idy3)
        for x in range(minx, maxx):
            for y in range(miny, maxy):
                # S1 = triArea(np.array([o_idx0, o_idy0]), np.array([o_idx1, o_idy1]), np.array([x, y]))
                # + triArea(np.array([o_idx2, o_idy2]), np.array([o_idx1, o_idy1]), np.array([x, y]))
                # + triArea(np.array([o_idx3, o_idy3]), np.array([o_idx2, o_idy2]), np.array([x, y]))
                # + triArea(np.array([o_idx0, o_idy0]), np.array([o_idx3, o_idy3]), np.array([x, y]))
                # if(S1-S < 0.001):
                #     map[x][y] = 0
                map[x][y] = 0
    return map
    # a=globalvar.obstacles_[0][1].x


global map
map = costmap()


def PlanAStarPath():
    vehicle_TPBV_ = globalvar.vehicle_TPBV_
    motion = get_motion_model()
    completeness_flag = 0
    start_node = Node(calc_xy_index(vehicle_TPBV_.x0, 0),
                      calc_xy_index(vehicle_TPBV_.y0, 1), 0.0, -1)
    goal_node = Node(calc_xy_index(vehicle_TPBV_.xtf, 0),
                     calc_xy_index(vehicle_TPBV_.ytf, 1), 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[calc_grid_index(start_node)] = start_node

    while 1:
        if len(open_set) == 0:
            [xx, yy] = search_closest(
                closed_set.keys(), goal_node.x, goal_node.y, start_node.x, start_node.y)
            closest_node = closed_set[yy * astar_.num_nodes_x + xx]
            x, y, path_length = calc_final_path(closest_node, closed_set)
            theta = len(x)*[0]
            x.reverse()
            y.reverse()
            theta.reverse()
            print(path_length)
            return [x, y, theta, path_length, completeness_flag]
        # id is the transformed coordinate, node is the key value
        c_id = min(
            open_set, key=lambda o: open_set[o].cost + calc_heuristic(goal_node, open_set[o]))
        current = open_set[c_id]

        if current.x == goal_node.x and current.y == goal_node.y:
            completeness_flag = 1
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove the item from the open set
        del open_set[c_id]

        # Add it to the closed set
        closed_set[c_id] = current

        # expand_grid search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_grid_index(node)
            # If the node is not safe, do nothing
            if not verify_node(node.x, node.y):
                continue
            if n_id in closed_set:
                continue

            if n_id not in open_set:
                open_set[n_id] = node  # discovered a new node
            else:
                if open_set[n_id].cost > node.cost:
                    # This path is the best until now. record it
                    open_set[n_id] = node
    x, y, path_length = calc_final_path(goal_node, closed_set)
    theta = len(x)*[0]
    x.reverse()
    y.reverse()
    theta.reverse()
    return [x, y, theta, path_length, completeness_flag]
