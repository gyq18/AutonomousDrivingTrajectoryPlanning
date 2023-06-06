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
# OBBhttps://blog.csdn.net/silangquan/article/details/50812425

# Vector number multiplication


def vclass_number_multiplication(a, v):
    v1 = vclass()
    v1.x = v.x*a
    v1.y = v.y*a
    return v1

# Vector dot product


def InnerProd(V1, V2):
    return (V1.x*V2.x+V1.y*V2.y)

#  OBB projection quadrilateral collision detection


def OBB_Quadrilateral_Collision(V1, V2):
    # coordinate position of the center of rectangle A
    PA = vclass()
    PA.x = (V1.x[0]+V1.x[1]+V1.x[2]+V1.x[3])/4
    PA.y = (V1.y[0]+V1.y[1]+V1.y[2]+V1.y[3])/4
    # unit vector representing the local x-axis of A
    l1 = math.sqrt((V1.x[3]-V1.x[0])**2+(V1.y[3]-V1.y[0])**2)
    Ax = vclass()
    Ax.x = (V1.x[3] - V1.x[0]) / l1
    Ax.y = (V1.y[3] - V1.y[0]) / l1
    # unit vector representing the local y-axis of A
    l2 = math.sqrt((V1.x[1]-V1.x[0])**2+(V1.y[1]-V1.y[0])**2)
    Ay = vclass()
    Ay.x = (V1.x[1] - V1.x[0]) / l2
    Ay.y = (V1.y[1] - V1.y[0]) / l2
    # half width of A (corresponds with the local x-axis of A)
    WA = l1/2
    # half height of A (corresponds with the local y-axis of A)
    HA = l2/2

    # coordinate position of the center of rectangle B
    PB = vclass()
    PB.x = (V2.x[0]+V2.x[1]+V2.x[2]+V2.x[3]) / 4
    PB.y = (V2.y[0]+V2.y[1]+V2.y[2]+V2.y[3]) / 4
    # unit vector representing the local x-axis of B
    l3 = math.sqrt((V2.x[3]-V2.x[0])**2+(V2.y[3]-V2.y[0])**2)
    Bx = vclass()
    Bx.x = (V2.x[3] - V2.x[0]) / l3
    Bx.y = (V2.y[3] - V2.y[0]) / l3
    # unit vector representing the local y-axis of B
    l4 = math.sqrt((V2.x[1]-V2.x[0])**2+(V2.y[1]-V2.y[0])**2)
    By = vclass()
    By.x = (V2.x[1] - V2.x[0]) / l4
    By.y = (V2.y[1] - V2.y[0]) / l4
    # half width of A (corresponds with the local x-axis of B)
    WB = l3 / 2
    # half height of A (corresponds with the local y-axis of B)
    HB = l4 / 2

    # unit vector representing the local x-axis of B
    l5 = math.sqrt((V2.x[3]-V2.x[2])**2+(V2.y[3]-V2.y[2])**2)
    Bx2 = vclass()
    Bx2.x = (V2.x[3] - V2.x[2]) / l3
    Bx2.y = (V2.y[3] - V2.y[2]) / l3
    # unit vector representing the local y-axis of B
    l6 = math.sqrt((V2.x[1]-V2.x[2])**2+(V2.y[1]-V2.y[2])**2)
    By2 = vclass()
    By2.x = (V2.x[1] - V2.x[2]) / l4
    By2.y = (V2.y[1] - V2.y[2]) / l4
    # half width of A (corresponds with the local x-axis of B)
    WB2 = l5 / 2
    # half height of A (corresponds with the local y-axis of B)
    HB2 = l6 / 2

    T = vclass()
    T.x = PB.x - PA.x
    T.y = PB.y-PA.y

    # Separating Axis Judge
    Separating_Axis = 0

    # CASE 1:
    # L = Ax
    # | T • Ax | > WA + | ( WB*Bx ) • Ax | + |( HB*By ) • Ax |
    # If true, there is a separating axis parallel Ax.
    if (math.fabs(InnerProd(T, Ax)) >
        WA + math.fabs(InnerProd(Ax, vclass_number_multiplication(WB, Bx)))
            + math.fabs(InnerProd(Ax, vclass_number_multiplication(HB, By)))):
        Separating_Axis = Separating_Axis+1

    # CASE 2:
    # L = Ay
    # | T • Ay | > HA + | ( WB*Bx ) • Ay | + |( HB*By ) • Ay |
    # If true, there is a separating axis parallel Ax.
    if (math.fabs(InnerProd(T, Ay)) >
        HA + math.fabs(InnerProd(vclass_number_multiplication(WB, Bx), Ay))
            + math.fabs(InnerProd(Ay, vclass_number_multiplication(HB, By)))):
        Separating_Axis = Separating_Axis+1

    # CASE 3:
    # L = Bx
    # | T • Bx | > | ( WA* Ax ) • Bx | + | ( HA*Ay ) • Bx | + WB
    # If true, there is a separating axis parallel Bx.
    if (math.fabs(InnerProd(T, Bx)) > WB + math.fabs(InnerProd(Bx, vclass_number_multiplication(WA, Ax))) + math.fabs(InnerProd(Bx, vclass_number_multiplication(HA, Ay)))):
        Separating_Axis = Separating_Axis+1

    # CASE 4:
    # L = By
    # | T • By | > | ( WA* Ax ) • By | + | ( HA*Ay ) • By | + HB
    # If true, there is a separating axis parallel By.
    if (math.fabs(InnerProd(T, By)) > HB + math.fabs(InnerProd(By, vclass_number_multiplication(WA, Ax))) + math.fabs(InnerProd(By, vclass_number_multiplication(HA, Ay)))):
        Separating_Axis = Separating_Axis + 1

    if (Separating_Axis > 0):
        return False
    else:
        return True

# OBB collision detection


def Vehicle_OBB_Collision(x, y, theta, obj):
    # Get vehicle Edge Data
    V = CreateVehiclePolygon(x, y, theta)
    return OBB_Quadrilateral_Collision(V, obj)


def CheckByOBB(x, y, theta):
    for obj in globalvar.obstacles_[0]:
        if(Vehicle_OBB_Collision(x, y, theta, obj) == 1):
            return True
    return False
