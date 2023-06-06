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
# from main_unstructure import triArea


def triArea(p1: np.array, p2: np.array, p3: np.array) -> float:
    a = norm(p1-p2)
    b = norm(p1-p3)
    c = norm(p2-p3)

    half = (a+b+c)/2
    area = math.sqrt(half*(half-a)*(half-b)*(half-c))

    return area


def Circumscribed_Circle(p, r, o1, o2):
    # First, determine whether o1o2 is within the circumscribed circle
    if (math.sqrt((p.x-o1.x)**2+(p.y-o1.y)**2) <= r):
        return True
    if (math.sqrt((p.x-o2.x)**2+(p.y-o2.y)**2) <= r):
        return True
    # If o1o2 they are all outside the obstacle, determine whether the circle intersects the line segment
    norm_dist = math.sqrt((o2.x - o1.x)**2+(o2.y - o1.y)**2)
    if (norm_dist < 1e-06):
        return False
    else:
        A = -(o2.y - o1.y)
        B = o2.x - o1.x
        # Distance from circle center to line segment
        dist = math.fabs(A * (p.x - o1.x) + B * (p.y - o1.y)
                         ) / math.sqrt(A**2 + B**2)
        if (dist > r):
            return False
        else:
            m = B * p.x - A * p.y
            # Calculate the coordinates of the intersection of the vertical line of the center of the circle and the straight line o1o2
            x4 = ((A**2) * o1.x + B * (m + o1.y * A)) / (A**2 + B**2)
            if(math.fabs(B) > 1e-06):
                if ((o1.x - x4) * (o2.x - x4) < 0):
                    return True
            else:
                y4 = (B * x4 - m) / A
                if((o1.y - y4) * (o2.y - y4) < 0):
                    return True
    return False

# Whether the front and rear circumscribed circles of the vehicle collide with obstacles


def Vehicle_Circumscribed_Circle_Collision_obj(x, y, theta, obj):
    vehicle_geometrics_ = globalvar.vehicle_geometrics_
    ncorner = 4
    Is_Collision = False
    # The center of the front circle of the vehicle is p1, the center of the rear circle is p2, and the radius of the circumscribed circle is r
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    r = math.sqrt((vehicle_geometrics_.vehicle_length / 4) **
                  2 + (vehicle_geometrics_.vehicle_width / 2)**2)
    p1 = vclass()
    p2 = vclass()
    p1.x = x + (vehicle_geometrics_.vehicle_length * 3 / 4 -
                vehicle_geometrics_.vehicle_rear_hang) * cos_theta
    p1.y = y + (vehicle_geometrics_.vehicle_length * 3 / 4 -
                vehicle_geometrics_.vehicle_rear_hang) * sin_theta

    p2.x = x + (vehicle_geometrics_.vehicle_length / 4 -
                vehicle_geometrics_.vehicle_rear_hang) * cos_theta
    p2.y = y + (vehicle_geometrics_.vehicle_length / 4 -
                vehicle_geometrics_.vehicle_rear_hang) * sin_theta

    o1 = vclass()
    o2 = vclass()
    for i in range(0, ncorner):
        o1.x = obj.x[i]
        o1.y = obj.y[i]
        o2.x = obj.x[(i+1) % ncorner]
        o2.y = obj.y[(i+1) % ncorner]
        Is_Collision1 = Circumscribed_Circle(p1, r, o1, o2)
        Is_Collision2 = Circumscribed_Circle(p2, r, o1, o2)
        if(Is_Collision1 or Is_Collision2):
            return True
    if(check_in(x, y, obj)):
        return True
    return Is_Collision

# Whether the detection point is in the obstacle


def check_in(x, y, obj):
    ncorner = 4
    obj_area = 0
    p_area = 0
    for i in range(0, ncorner, 2):
        p1 = np.array([[obj.x[i % ncorner]], [obj.y[i % ncorner]]])
        p2 = np.array([[obj.x[(i + 1) % ncorner]], [obj.y[(i + 1) % ncorner]]])
        p3 = np.array([[obj.x[(i + 2) % ncorner]], [obj.y[(i + 2) % ncorner]]])
        obj_area = obj_area + triArea(p1, p2, p3)
    for i in range(0, ncorner):
        p1 = np.array([[x], [y]])
        p2 = np.array([[obj.x[i % ncorner]], [obj.y[i % ncorner]]])
        p3 = np.array([[obj.x[(i + 1) % ncorner]], [obj.y[(i + 1) % ncorner]]])
        p_area = p_area + triArea(p1, p2, p3)

    if (p_area > obj_area + 0.1):
        return 0
    else:
        return 1


def CheckByCircle(x, y, theta):
    for obj in globalvar.obstacles_[0]:
        if(Vehicle_Circumscribed_Circle_Collision_obj(x, y, theta, obj) == True):
            return True
    return False