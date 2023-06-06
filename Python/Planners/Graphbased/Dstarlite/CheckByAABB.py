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

from main_unstructure import CreateVehiclePolygon
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



def Vehicle_AABB_Collision(x,y, theta, obj):
    #  Get vehicle Edge Data
    V = CreateVehiclePolygon(x, y, theta)
    # print(V.x)
    # print(V.y)
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
    # print(V1.x)
    # print(V1.y)
    # print(V2.x)
    # print(V2.y)
    # Judge whether there is collision between projections
    return AABB_Quadrilateral_Collision(V1, V2)


def Vehicle_Allobj_AABB_Collision(x,y, theta, objs):
    for obj in objs[0]:
        if(Vehicle_AABB_Collision(x,y, theta, obj)==1):
            return True
    return False
# p=vclass()
# p.x=0
# p.y=0
# theta=0
# obj=vclass()
# obj.x=np.array([1,1,3,3])
# obj.y=np.array([0,2,2,1])
# print(Vehicle_AABB_Collision(p, theta, obj))
