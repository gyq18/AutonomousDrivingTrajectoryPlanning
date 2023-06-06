import warnings
import math
from joblib import parallel_backend
from matplotlib.pyplot import flag
import numpy as np

from random import random as rand
from matplotlib.path import Path
from numpy.linalg import norm as norm
from sqlalchemy import false, true

import globalvar
from globalvar import vclass

# from main_unstructure import CreateVehiclePolygon
# from main_unstructure import triArea

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


def triArea(p1: np.array, p2: np.array, p3: np.array):
    a = norm(p1-p2)
    b = norm(p1-p3)
    c = norm(p2-p3)

    half = (a+b+c)/2
    area = math.sqrt(half*(half-a)*(half-b)*(half-c))

    return area

# Collision detection of point p and obstacle V by comparative area method


def P_Comparative_Area_Collision(x, y, obj):
    ncorner = 4
    obj_area = 0
    for i in range(0, ncorner, 2):
        p1 = np.array([[obj.x[i % ncorner]], [obj.y[i % ncorner]]])
        p2 = np.array([[obj.x[(i + 1) % ncorner]], [obj.y[(i + 1) % ncorner]]])
        p3 = np.array([[obj.x[(i + 2) % ncorner]], [obj.y[(i + 2) % ncorner]]])
        obj_area = obj_area + triArea(p1, p2, p3)
    p_area = 0
    for i in range(0, ncorner):
        p1 = np.array([[x], [y]])
        p2 = np.array([[obj.x[i % ncorner]], [obj.y[i % ncorner]]])
        p3 = np.array([[obj.x[(i + 1) % ncorner]], [obj.y[(i + 1) % ncorner]]])
        p_area = p_area + triArea(p1, p2, p3)
    if (p_area > obj_area + 0.1):
        return False
    else:
        return True

# Collision detection by comparative area method
# Algorithm reference https://zhuanlan.zhihu.com/p/449795053


def Comparative_Area_Collision_obj(x, y, theta, obj):
    # Get vehicle Edge Data
    V = CreateVehiclePolygon(x, y, theta)
    Is_Collision = False
    ncorner = 4
    for i in range(0, ncorner):
        Is_Collision = P_Comparative_Area_Collision(V.x[i], V.y[i], obj)
        if (Is_Collision == True):
            return True
    for i in range(0, ncorner):
        Is_Collision = P_Comparative_Area_Collision(obj.x[i], obj.y[i], V)
        if (Is_Collision == true):
            return True
    return False


def CheckByArea(x, y, theta):
    for obj in globalvar.obstacles_[0]:
        if(Comparative_Area_Collision_obj(x, y, theta, obj) == True):
            return True
    return False
