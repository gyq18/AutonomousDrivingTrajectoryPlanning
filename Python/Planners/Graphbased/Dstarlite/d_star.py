# reference resources: https://blog.csdn.net/lqzdreamer/article/details/85055569

import math
from sys import maxsize


import math

import matplotlib.pyplot as plt

import numpy as np

import time
from numpy.linalg import norm as norm
from pyrsistent import v
from sqlalchemy import true

import globalvar
from globalvar import vclass
from globalvar import hybrid_astar_Set

from main_unstructure import CreateVehiclePolygon
from main_unstructure import inpolygon

from AABB import Vehicle_Allobj_AABB_Collision
from Circumscribed_circle import Vehicle_Allobj_Circumscribed_Circle_Collision
import time

global d_star_lite_
d_star_lite_ = hybrid_astar_Set()

# Convert actual distance to index value
# 实际距离转换为索引值


def calc_xy_index(position, dim):
    if(dim == 0):
        pos = math.ceil(position / d_star_lite_.resolution_x)
    if(dim == 1):
        pos = math.ceil(position / d_star_lite_.resolution_y)
    return pos


# Convert index value to actual distance
# 索引值转换为实际距离


def calc_grid_position(index, dim):
    """
    calc grid position

    :param index:
    :param min_position:
    :return:
    """
    if(dim == 0):
        pos = index * d_star_lite_.resolution_x
    if(dim == 1):
        pos = index * d_star_lite_.resolution_y
    return pos


class State(object):

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = 0
        self.t = "new"
        self.h = 0
        self.k = 0  # k即为f

    def cost(self, state):
        if (self.state > 0 and self.state <= 2) or (state.state > 0 and state.state <= 2):
            return maxsize  # 存在障碍物时，距离无穷大
        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):

        self.state = state


class Map(object):
    '''
    Create map
    创建地图
    '''
    def __init__(self):
        self.row = round(d_star_lite_.num_nodes_x)
        self.col = round(d_star_lite_.num_nodes_y)
        self.map = self.init_map()
        map1 = np.zeros((round(d_star_lite_.num_nodes_x),
                         round(d_star_lite_.num_nodes_y)))
        # Set up obstacles
        # 设置障碍物
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
            o_idx = ([o_idx0, o_idx0, o_idx1, o_idx2, o_idx3])
            o_idy = ([o_idy0, o_idy0, o_idy1, o_idy2, o_idy3])
            for x in range(minx, maxx+1):
                for y in range(miny, maxy+1):
                    if(inpolygon((x), (y), o_idx, o_idy)):
                        map1[x][y] = 1
                        self.set_obstacle(x, y)

        # Set up border
        # 设置边框
        maxx = round(d_star_lite_.num_nodes_x)
        maxy = round(d_star_lite_.num_nodes_y)
        for x in range(maxx):
            map1[x][0] = 1
            self.set_obstacle(x, 0)
            map1[x][maxy - 1] = 1
            self.set_obstacle(x, maxy - 1)
        for y in range(maxy):
            map1[0][y] = 1
            self.set_obstacle(0, y)
            map1[maxx - 1][y] = 1
            self.set_obstacle(maxx - 1, y)

        # Set the point where the vehicle will collide
        # 设置车辆将会有碰撞的点
        x_r = math.ceil((globalvar.vehicle_geometrics_.vehicle_wheelbase +
                        globalvar.vehicle_geometrics_.vehicle_front_hang) / d_star_lite_.resolution_x) + 1
        x_l = math.ceil(
            globalvar.vehicle_geometrics_.vehicle_rear_hang / d_star_lite_.resolution_x) + 1
        dy = math.ceil(globalvar.vehicle_geometrics_.vehicle_width /
                       2 / d_star_lite_.resolution_y) + 1
        for x in range(maxx):
            for y in range(maxy):
                if (map1[x][y] == 1):
                    for m in range(x - x_r - 1, x + x_l + 1):
                        if(m >= 0 and m < maxx):
                            for n in range(y - dy - 1, y + dy + 1):
                                if(n >= 0 and n < maxy and map1[m][n] == 0):
                                    map1[m][n] = 2
                                    self.set_obstacle(m, n)

    def init_map(self):
        map_list = []
        for i in range(self.row):
            cur = []
            for j in range(self.col):
                cur.append(State(i, j))
            map_list.append(cur)
        return map_list

    def get_neighbers(self, state):
        # Get 8 neighborhood
        # 获取8邻域
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    # Set up obstacles
    # 设置障碍物
    def set_obstacle(self, x, y):
        if x < 0 or x >= self.row or y < 0 or y >= self.col:
            return
        self.map[x][y].set_state(1)


class Dstar:

    def __init__(self):
        self.map = Map()
        # Create an empty collection
        # 创建空集合
        self.open_list = set()
        self.start_x = calc_xy_index(globalvar.vehicle_TPBV_.x0, 0)
        self.start_y = calc_xy_index(globalvar.vehicle_TPBV_.y0, 1)
        self.end_x = calc_xy_index(globalvar.vehicle_TPBV_.xtf, 0)
        self.end_y = calc_xy_index(globalvar.vehicle_TPBV_.ytf, 1)

    # Add obstacles
    # 增加障碍物
    def add_obstacle(self, obj):
        globalvar.Nobs = globalvar.Nobs+1
        globalvar.obstacles_[0].append(obj)
        map1 = np.zeros((round(d_star_lite_.num_nodes_x),
                         round(d_star_lite_.num_nodes_y)))
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
        o_idx = ([o_idx0, o_idx0, o_idx1, o_idx2, o_idx3])
        o_idy = ([o_idy0, o_idy0, o_idy1, o_idy2, o_idy3])
        for x in range(minx, maxx+1):
            for y in range(miny, maxy+1):
                if(inpolygon((x), (y), o_idx, o_idy)):
                    map1[x][y] = 1
                    self.map.set_obstacle(x, y)

        # Set the point where the vehicle will collide
        # 设置车辆将会有碰撞的点
        x_r = math.ceil((globalvar.vehicle_geometrics_.vehicle_wheelbase +
                        globalvar.vehicle_geometrics_.vehicle_front_hang) / d_star_lite_.resolution_x) + 1
        x_l = math.ceil(
            globalvar.vehicle_geometrics_.vehicle_rear_hang / d_star_lite_.resolution_x) + 1
        dy = math.ceil(globalvar.vehicle_geometrics_.vehicle_width /
                       2 / d_star_lite_.resolution_y) + 1
        for x in range(minx, maxx+1):
            for y in range(miny, maxy+1):
                if (map1[x][y] == 1):
                    for m in range(x - x_r - 1, x + x_l + 1):
                        if(m >= 0 and m < maxx):
                            for n in range(y - dy - 1, y + dy + 1):
                                if(n >= 0 and n < maxy and map1[m][n] == 0):
                                    map1[m][n] = 2
                                    self.map.set_obstacle(m, n)

    def process_state(self):
        '''
        Main process of D * algorithm
        D*算法的主要过程
        :return:
        '''
        #Get the minimum k node in the open list
        # 获取open list列表中最小k的节点
        x = self.min_state()
        if x is None:
            return -1
        #Get the k value of the smallest k node in the open list
        # 获取open list列表中最小k节点的k值
        k_old = self.get_kmin()
        # Remove from openlist
        # 从openlist中移除
        self.remove(x)
        if k_old < x.h:
            for y in self.map.get_neighbers(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbers(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbers(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(x, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        # 获取openlist中k值最小对应的节点
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        # Get the k with the smallest k(f) value in the openlist table
        # 获取openlist表中k(f)值最小的k
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        # It uses an openlist to recurse the cost on the whole path through the parent
        # 是以一个openlist，通过parent递推整条路径上的cost
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def modify(self, state):
        '''
        When the obstacle changes, push back from the target point to the starting point 
        to update the change of path cost caused by the change of obstacle
        当障碍物发生变化时，从目标点往起始点回推，更新由于障碍物发生变化而引起的路径代价的变化
        :param state:
        :return:
        '''
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if((k_min == -1) or (k_min >= state.h)):
                break

    def replan(self):
        time_start = time.time()
        self.Is_complete = False
        start_idx_x = self.start_x
        start_idx_y = self.start_y
        end_idx_x = self.end_x
        end_idx_y = self.end_y
        start = self.map.map[start_idx_x][start_idx_y]
        end = self.map.map[end_idx_x][end_idx_y]

        self.x_id = []
        self.y_id = []
        self.open_list.add(end)
        while True:
            self.process_state()
            if start.t == "close":
                break
        self.x_id.append(start_idx_x)
        self.y_id.append(start_idx_y)
        if(start.state > 0):
            self.Is_complete = False
            return
        cur = start
        '''
        Start from the starting point and travel to the target point. When encountering obstacles,
        modify the cost again and find the path again
        从起始点开始，往目标点行进，当遇到障碍物时，重新修改代价，再寻找路径
        '''
        while cur != end:
            time_end = time.time()
            if(time_end-time_start>10):
                return self.Is_complete
            if(cur.state > 0):
                self.Is_complete = False
                return self.Is_complete
            self.x_id.append(cur.x)
            self.y_id.append(cur.y)
            if cur.parent.state > 0 and cur.parent.state <= 2:
                self.modify(cur)
                continue
            cur = cur.parent

        if(cur == end):
            self.Is_complete = True
            self.x_id.append(end_idx_x)
            self.y_id.append(end_idx_y)
        return self.Is_complete

    def get_path(self):
        x = []
        y = []
        theta = []
        length = 0
        # Convert matrix index to actual map1
        # 矩阵索引转为实际地图
        for (rx, ry) in zip(self.x_id, self.y_id):
            act_x = calc_grid_position(rx, 0)
            act_y = calc_grid_position(ry, 0)
            x.append(act_x)
            y.append(act_y)
            length = length + math.sqrt((x[0] - x[-1])**2 + (y[0] - y[-1])**2)
            theta.append(0)
        return [x, y, theta, length, self.Is_complete]
