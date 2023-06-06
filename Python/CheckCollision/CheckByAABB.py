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


# Calculates a set of non adjacent points of a projected quadrilateral
def Min(V):
    X = V.x
    Y = V.y
    minx = V.x[0]
    miny = V.y[0]
    for i in X:
        if (i < minx):
            minx = i
    for i in Y:
        if (i < miny):
            miny = i
    m = vclass()
    m.x = minx
    m.y = miny
    return m


def Max(V):
    X = V.x
    Y = V.y
    maxx = V.x[0]
    maxy = V.y[0]
    for i in X:
        if (i > maxx):
            maxx = i
    for i in Y:
        if (i > maxy):
            maxy = i
    m = vclass()
    m.x = maxx
    m.y = maxy
    return m

# AABB projection quadrilateral collision detection


def AABB_Quadrilateral_Collision(V1, V2):
    IsCollisionx = 0
    IsCollisiony = 0
    for i in range(0, 2):
        # X-axis
        if ((V1.x[i] - V2.x[0]) * (V1.x[i] - V2.x[1]) <= 0):
            IsCollisionx = 1
        # Y-axis
        if ((V1.y[i] - V2.y[0]) * (V1.y[i] - V2.y[1]) <= 0):
            IsCollisiony = 1
    for i in range(0, 2):
        if ((V2.x[i] - V1.x[0]) * (V2.x[i] - V1.x[1]) <= 0):
            IsCollisionx = 1
        if ((V2.y[i] - V1.y[0]) * (V2.y[i] - V1.y[1]) <= 0):
            IsCollisiony = 1
    if (IsCollisionx == 1 and IsCollisiony == 1):
        return True
    else:
        return False

# AABB collision detection


def Vehicle_AABB_Collision(x, y, theta, obj):
    #  Get vehicle Edge Data
    V = CreateVehiclePolygon(x, y, theta)
    # Set vehicle projection data

    A = vclass()
    B = vclass()
    A = Min(V)
    B = Max(V)

    V1 = vclass()
    V1.x = np.array([A.x, B.x])
    V1.y = np.array([A.y, B.y])

    # Set obj projection data

    C = vclass()
    D = vclass()
    C = Min(obj)
    D = Max(obj)

    V2 = vclass()
    V2.x = np.array([C.x, D.x])
    V2.y = np.array([C.y, D.y])
    # Judge whether there is collision between projections
    return AABB_Quadrilateral_Collision(V1, V2)


def CheckByAABB(x, y, theta):
    for obj in globalvar.obstacles_[0]:
        if(Vehicle_AABB_Collision(x, y, theta, obj) == 1):
            return True
    return False
