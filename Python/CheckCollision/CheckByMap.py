import math
from math import ceil
from joblib import parallel_backend
from matplotlib.pyplot import flag
import numpy as np
import cv2
import time

from random import random as rand
from matplotlib.path import Path
from numpy.linalg import norm as norm
from sqlalchemy import false, true

import globalvar
from globalvar import hybrid_astar_Set, planning_scale_Set, vclass
# %% vehicle settings
# vehicle geometrics settings
global planning_scale_
planning_scale_ = globalvar.planning_scale_
global hybrid_astar_
hybrid_astar_ = globalvar.hybrid_astar_
global Nobs
Nobs = globalvar.Nobs
global vehicle_TPBV_
vehicle_TPBV_ = globalvar.vehicle_TPBV_
global vehicle_geometrics_
vehicle_geometrics_ = globalvar.vehicle_geometrics_
global margin_obs_
margin_obs_ = globalvar.margin_obs_


def CheckByMap(x, y, theta):
    costmap = globalvar.costmap_
    Is_collision = False
    if costmap is None or costmap.any() == 0:
        CreateDilatedCostmap(globalvar.obstacles_)
    
    # The center of the front circle of the vehicle is p1, the center of the rear circle is p2, and the radius of the circumscribed circle is r
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    # Get vehicle Edge Data
    p1x = x + (vehicle_geometrics_.vehicle_length * 3/4 -
               vehicle_geometrics_.vehicle_rear_hang) * cos_theta
    p1y = y + (vehicle_geometrics_.vehicle_length * 3/4 -
               vehicle_geometrics_.vehicle_rear_hang) * sin_theta

    p2x = x + (vehicle_geometrics_.vehicle_length / 4 -
               vehicle_geometrics_.vehicle_rear_hang) * cos_theta
    p2y = y + (vehicle_geometrics_.vehicle_length / 4 -
               vehicle_geometrics_.vehicle_rear_hang) * sin_theta

    px = np.array([p1x, p2x])
    py = np.array([p1y, p2y])

    indx = np.round((px - planning_scale_.xmin) / hybrid_astar_.resolution_x)
    indy = np.round((py - planning_scale_.ymin) / hybrid_astar_.resolution_y)
    indx = np.int64(indx)
    indy = np.int64(indy)

    mapsize = globalvar.costmap_.shape
    indy = mapsize[1] - indy

    # from PIL import Image
    # costmap1=np.matrix(globalvar.costmap_)
    # costmap1[indy[0], indx[0]] = 255
    # costmap1[indy[1], indx[1]] = 255
    # new2 = Image.fromarray(costmap1 * 100)
    # new2.show()

    # Note that matrix conversion changes the array direction
    if (globalvar.costmap_[indy[0]][indx[0]] + globalvar.costmap_[indy[1]][indx[1]] > 0):
        Is_collision = True
    return Is_collision


def ConvertXYToIndex(x, y):
    ind1 = ceil((x - planning_scale_.xmin)/hybrid_astar_.resolution_x)
    ind2 = ceil((y - planning_scale_.ymin)/hybrid_astar_.resolution_y)
    if ind1 <= (hybrid_astar_.num_nodes_x-1) and ind1 >= 0 and ind2 <= (hybrid_astar_.num_nodes_y-1) and ind2 >= 0:
        return [ind1, ind2]
    if ind1 > hybrid_astar_.num_nodes_x-1:
        ind1 = hybrid_astar_.num_nodes_x-1
    elif ind1 < 0:
        ind1 = 0
    if ind2 > hybrid_astar_.num_nodes_y-1:
        ind2 = hybrid_astar_.num_nodes_y-1
    elif ind2 < 0:
        ind2 = 0
    return [ind1, ind2]

# Design 01 map
# %%


def CreateDilatedCostmap(obstacles_):
    xmin = planning_scale_.xmin
    ymin = planning_scale_.ymin
    resolution_x = hybrid_astar_.resolution_x
    resolution_y = hybrid_astar_.resolution_y
    costmap = np.zeros((int(hybrid_astar_.num_nodes_x),
                       int(hybrid_astar_.num_nodes_y)))
    Ny = int(hybrid_astar_.num_nodes_y)
    for i in range(Nobs):
        vx = obstacles_[0][i].x
        vy = obstacles_[0][i].y
        x_lb = min(vx)
        x_ub = max(vx)
        y_lb = min(vy)
        y_ub = max(vy)
        [Nmin_x, Nmin_y] = ConvertXYToIndex(x_lb, y_lb)
        [Nmax_x, Nmax_y] = ConvertXYToIndex(x_ub, y_ub)
        for j in range(Nmin_x, Nmax_x+1):
            for k in range(Nmin_y, Nmax_y+1):
                row = Ny-k-1
                col = j
                if costmap[row, col] == 1:
                    continue
                cur_x = xmin + j*resolution_x
                cur_y = ymin + k*resolution_y
                if inpolygon(cur_x, cur_y, obstacles_[0][i].x, obstacles_[0][i].y) == 1:
                    costmap[row, col] = 1
    for x in range(int(hybrid_astar_.num_nodes_x)):
        costmap[x, 0] = 1
        costmap[x, int(hybrid_astar_.num_nodes_y) - 1] = 1

    for y in range(int(hybrid_astar_.num_nodes_y)):
        costmap[0, y] = 1
        costmap[int(hybrid_astar_.num_nodes_x) - 1, y] = 1

    length_unit = 0.5*(resolution_x + resolution_y)
    r = ceil(vehicle_geometrics_.radius/length_unit) + 2
    basic_elem = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (r, r))
    globalvar.costmap_ = cv2.dilate(costmap, basic_elem)
    return


def inpolygon(xq, yq, xv, yv):
    """
    reimplement inpolygon in matlab
    :type xq: np.ndarray
    :type yq: np.ndarray
    :type xv: np.ndarray
    :type yv: np.ndarray
    """
    # Merge into an array of vertices
    vertices = np.vstack((xv, yv)).T
    # obj Path
    path = Path(vertices)

    # merge into an array of test point
    try:
        test_points = np.hstack(
            [xq.reshape(xq.size, -1), yq.reshape(yq.size, -1)])
    except:
        test_points = np.array([[xq, yq]])
    # get a mask to identify whether test_points are in the path
    _in = path.contains_points(test_points)
    # get a mask to identify whether test_points are in/on the path
    _in_on = path.contains_points(test_points, radius=-1e-10)
    _on = _in ^ _in_on
    return _in_on
