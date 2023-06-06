import math
from matplotlib.pyplot import flag
import numpy as np

from random import random as rand
from matplotlib.path import Path
from numpy.linalg import norm as norm

import time

from sqlalchemy import false
import globalvar
from globalvar import vclass


def triArea(p1: np.array, p2: np.array, p3: np.array):
    a = norm(p1-p2)
    b = norm(p1-p3)
    c = norm(p2-p3)

    half = (a+b+c)/2
    area = math.sqrt(half*(half-a)*(half-b)*(half-c))

    return area


def checkObj_point(xr=None, obj=None):
    # Determine if xr is inside the obstacle
    # The obstacle here is mainly a polygon with four vertices

    result = 0
    ncorner = 4
    # Calculate the area of the obstacle area_obj and the sum of the four triangles areaarea, if they are equal, it means xr is inside the obstacle
    area = 0
    area_obj = 0
    # triArea is used to calculate the area of a triangle
    for i in range(0, ncorner):
        area = area + triArea(xr, obj[:, i], obj[:, np.mod(i+1, ncorner)])

    for i in range(1, ncorner-1):
        area_obj = area_obj + \
            triArea(obj[:, 0], obj[:, i], obj[:, np.mod(i+1, ncorner)])

    # if the reference point is inside the obstacle, then area = polyarea
    if norm(area_obj - area) < 0.01:
        result = 1
    return result


def checkObj_linev(x1=None, x2=None, obj=None):
    result = 1
    # Number of obstacles
    n, __ = obj.shape
    nobj = int(n / 2)
    for i in range(0, nobj):
        # index = np.arange((i - 1) * 2 + 1,i * 2+1)
        temp_new_obj = obj[2*i:2*i+2, :]

        # First determine whether the vertex is inside the obstacle
        result1 = checkObj_point(x1, temp_new_obj)
        result2 = checkObj_point(x2, temp_new_obj)
        if result1 == 0 and result2 == 0:
            result = 0
        else:
            result = 1
            break
        # If the vertices are both outside the obstacle, determine whether the two diagonals intersect the line segment
    # Direction of the line segment
        v1 = x2 - x1
        # Diagonal1
        c1 = temp_new_obj[:, 0]
        c2 = temp_new_obj[:, 2]
        # Direction vector of the diagonal
        v2 = c2 - c1
        # If two lines are parallel, skip
        norm_dist1 = norm(v1 - v2)
        norm_dist2 = norm(v1 + v2)
        if norm_dist1 < 1e-06 or norm_dist2 < 1e-06:
            result = 0
        else:
            # Calculate the intersection of two lines
            t1 = (v2[1] * (c1[0] - x1[0]) + v2[0] * (x1[1] - c1[1])) / \
                (v1[0] * v2[1] - v2[0] * v1[1])
            t2 = (v1[1] * (x1[0] - c1[0]) + v1[0] * (c1[1] - x1[1])) / \
                (v2[0] * v1[1] - v1[0] * v2[1])
            if t1 >= 0 and t1 <= 1 and t2 >= 0 and t2 <= 1:
                result = 1
                break
        # Diagonal2
        c1 = temp_new_obj[:, 1]
        c2 = temp_new_obj[:, 3]
        # Direction vector of the diagonal
        v2 = c2 - c1
        # If two lines are parallel, skip
        norm_dist1 = norm(v1 - v2)
        norm_dist2 = norm(v1 + v2)
        if norm_dist1 < 1e-06 or norm_dist2 < 1e-06:
            result = 0
        else:
            # Calculate the intersection of two lines
            t1 = (v2[1] * (c1[0] - x1[0]) + v2[0] * (x1[1] - c1[1])) / \
                (v1[0] * v2[1] - v2[0] * v1[1])
            t2 = (v1[1] * (x1[0] - c1[0]) + v1[0] * (c1[1] - x1[1])) / \
                (v2[0] * v1[1] - v1[0] * v2[1])
            if t1 >= 0 and t1 <= 1 and t2 >= 0 and t2 <= 1:
                result = 1
                break

    return result

# vehicle geometrics settings


def CreateVehiclePolygon(x, y, theta):
    vehicle_geometrics_ = globalvar.vehicle_geometrics_
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5
    AX = x + (vehicle_geometrics_.vehicle_front_hang +
              vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta
    BX = x + (vehicle_geometrics_.vehicle_front_hang +
              vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta
    CX = x - vehicle_geometrics_.vehicle_rear_hang * \
        cos_theta + vehicle_half_width * sin_theta
    DX = x - vehicle_geometrics_.vehicle_rear_hang * \
        cos_theta - vehicle_half_width * sin_theta
    AY = y + (vehicle_geometrics_.vehicle_front_hang +
              vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta
    BY = y + (vehicle_geometrics_.vehicle_front_hang +
              vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta
    CY = y - vehicle_geometrics_.vehicle_rear_hang * \
        sin_theta - vehicle_half_width * cos_theta
    DY = y - vehicle_geometrics_.vehicle_rear_hang * \
        sin_theta + vehicle_half_width * cos_theta
    V = vclass()
    V.x = np.array([AX, BX, CX, DX, AX])
    V.y = np.array([AY, BY, CY, DY, AY])
    return V

# Collision detection by checking for lines and vertices of the gnerated rectangle (vehicle)


def CheckByLine(x, y, theta):
    Is_collision = False
    # Get vehicle Edge Data
    VV = CreateVehiclePolygon(x, y, theta)
    x = (VV.x).reshape((1, 5))
    y = (VV.y).reshape((1, 5))
    V = np.concatenate([x, y], axis=0)
    for i in range(globalvar.Nobs):
        objx = (globalvar.obstacles_[0][i].x).reshape((1, 4))
        objy = (globalvar.obstacles_[0][i].y).reshape((1, 4))
        obj = np.concatenate([objx, objy], axis=0)
        flag1 = checkObj_linev(V[:, 0], V[:, 1], obj)
        flag2 = checkObj_linev(V[:, 1], V[:, 2], obj)
        flag3 = checkObj_linev(V[:, 2], V[:, 3], obj)
        flag4 = checkObj_linev(V[:, 3], V[:, 0], obj)
        if flag1+flag2+flag3+flag4 > 0:
            Is_collision = True
        if Is_collision:
            break
    return Is_collision
