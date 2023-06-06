# This code is for testing whether the trajectory is valid
# ==============================================================================
import math
from joblib import parallel_backend
from matplotlib.pyplot import flag
import numpy as np

from random import random as rand
from matplotlib.path import Path
from numpy.linalg import norm as norm

import globalvar
from globalvar import vclass
#%% vehicle settings
# vehicle geometrics settings


def CreateVehiclePolygon(x,y,theta):
    vehicle_geometrics_ = globalvar.vehicle_geometrics_
    cos_theta = math.cos( theta )
    sin_theta = math.sin( theta )
    vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5
    AX = x + ( vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase ) * cos_theta - vehicle_half_width * sin_theta
    BX = x + ( vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase ) * cos_theta + vehicle_half_width * sin_theta
    CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta
    DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta
    AY = y + ( vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase ) * sin_theta + vehicle_half_width * cos_theta
    BY = y + ( vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase ) * sin_theta - vehicle_half_width * cos_theta
    CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta
    DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta
    V = vclass()
    V.x = np.array([ AX, BX, CX, DX, AX ])
    V.y = np.array([ AY, BY, CY, DY, AY ])
    return V
#%% generate random obstacles: obstacles do not overlap with the vehicle's initial/terminal state

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
        test_points = np.hstack([xq.reshape(xq.size, -1), yq.reshape(yq.size, -1)])
    except:
        test_points=np.array([[xq,yq]])
    # 得到一个test_points是否严格在path内的mask，是bool值数组
    _in = path.contains_points(test_points)
    # 得到一个test_points是否在path内部或者在路径上的mask
    _in_on = path.contains_points(test_points, radius=-1e-10)
    # 得到一个test_points是否在path路径上的mask
    _on = _in ^ _in_on
    
    # return _in_on, _on
    return _in_on

def triArea(p1:np.array, p2:np.array, p3:np.array) -> float:
    a = norm(p1-p2)
    b = norm(p1-p3)
    c = norm(p2-p3)

    half = (a+b+c)/2
    area = math.sqrt(half*(half-a)*(half-b)*(half-c))

    return area

def checkObj_point(xr = None,obj = None): 
    ## Determine if xr is inside the obstacle
    # The obstacle here is mainly a polygon with four vertices
    
    result = 0
    ncorner = 4
    # Calculate the area of the obstacle area_obj and the sum of the four triangles areaarea, if they are equal, it means xr is inside the obstacle
    area = 0
    area_obj = 0
    # triArea is used to calculate the area of a triangle
    for i in range(0,ncorner):
        area = area + triArea(xr,obj[:,i],obj[:,np.mod(i+1,ncorner)])

    for i in range(1,ncorner-1):
        area_obj = area_obj + triArea(obj[:,0],obj[:,i],obj[:,np.mod(i+1,ncorner)])

    # if the reference point is inside the obstacle, then area = polyarea
    if norm(area_obj - area) < 0.01:
        result = 1
    return result

def checkObj_linev(x1 = None,x2 = None,obj = None):
    result = 1
    # Number of obstacles
    n,__ = obj.shape
    nobj = int(n / 2)
    for i in range(0,nobj):
        # index = np.arange((i - 1) * 2 + 1,i * 2+1)
        temp_new_obj = obj[2*i:2*i+2,:]

        ## First determine whether the vertex is inside the obstacle
        result1 = checkObj_point(x1,temp_new_obj)
        result2 = checkObj_point(x2,temp_new_obj)
        if result1 == 0 and result2 == 0:
            result = 0
        else:
            result = 1
            break
        ## If the vertices are both outside the obstacle, determine whether the two diagonals intersect the line segment
    # Direction of the line segment
        v1 = x2 - x1
        ## Diagonal1
        c1 = temp_new_obj[:,0]
        c2 = temp_new_obj[:,2]
        # Direction vector of the diagonal
        v2 = c2 - c1
        # If two lines are parallel, skip
        norm_dist1 = norm(v1 - v2)
        norm_dist2 = norm(v1 + v2)
        if norm_dist1 < 1e-06 or norm_dist2 < 1e-06:
            result = 0
        else:
            # Calculate the intersection of two lines
            t1 = (v2[1] * (c1[0] - x1[0]) + v2[0] * (x1[1] - c1[1])) / (v1[0] * v2[1] - v2[0] * v1[1])
            t2 = (v1[1] * (x1[0] - c1[0]) + v1[0] * (c1[1] - x1[1])) / (v2[0] * v1[1] - v1[0] * v2[1])
            if t1 >= 0 and t1 <= 1 and t2 >= 0 and t2 <= 1:
                result = 1
                break
        ## Diagonal2
        c1 = temp_new_obj[:,1]
        c2 = temp_new_obj[:,3]
        # Direction vector of the diagonal
        v2 = c2 - c1
        # If two lines are parallel, skip
        norm_dist1 = norm(v1 - v2)
        norm_dist2 = norm(v1 + v2)
        if norm_dist1 < 1e-06 or norm_dist2 < 1e-06:
            result = 0
        else:
            # Calculate the intersection of two lines
            t1 = (v2[1] * (c1[0] - x1[0]) + v2[0] * (x1[1] - c1[1])) / (v1[0] * v2[1] - v2[0] * v1[1])
            t2 = (v1[1] * (x1[0] - c1[0]) + v1[0] * (c1[1] - x1[1])) / (v2[0] * v1[1] - v1[0] * v2[1])
            if t1 >= 0 and t1 <= 1 and t2 >= 0 and t2 <= 1:
                result = 1
                break
        # diagonal1
        v1 = x2 - x1
        ## Diagonal1
        c1 = temp_new_obj[:,0]
        c2 = temp_new_obj[:,2]
        # Direction vector of the diagonal
        v2 = c2 - c1
        # If two lines are parallel, skip
        norm_dist1 = norm(v1 - v2)
        norm_dist2 = norm(v1 + v2)
        if norm_dist1 < 1e-06 or norm_dist2 < 1e-06:
            result = 0
        else:
            # Calculate the intersection of two lines
            t1 = (v2[1] * (c1[0] - x1[0]) + v2[0] * (x1[1] - c1[1])) / (v1[0] * v2[1] - v2[0] * v1[1])
            t2 = (v1[1] * (x1[0] - c1[0]) + v1[0] * (c1[1] - x1[1])) / (v2[0] * v1[1] - v1[0] * v2[1])
            if t1 >= 0 and t1 <= 1 and t2 >= 0 and t2 <= 1:
                result = 1
                break
        ## Diagonal2
        c1 = temp_new_obj[:,1]
        c2 = temp_new_obj[:,3]
        # Direction vector of the diagonal
        v2 = c2 - c1
        # If two lines are parallel, skip
        norm_dist1 = norm(v1 - v2)
        norm_dist2 = norm(v1 + v2)
        if norm_dist1 < 1e-06 or norm_dist2 < 1e-06:
            result = 0
        else:
            # Calculate the intersection of two lines
            t1 = (v2[1] * (c1[0] - x1[0]) + v2[0] * (x1[1] - c1[1])) / (v1[0] * v2[1] - v2[0] * v1[1])
            t2 = (v1[1] * (x1[0] - c1[0]) + v1[0] * (c1[1] - x1[1])) / (v2[0] * v1[1] - v1[0] * v2[1])
            if t1 >= 0 and t1 <= 1 and t2 >= 0 and t2 <= 1:
                result = 1
                break
    return result

# def GenerateStaticObstacles_unstructured()->list[list]:
def GenerateStaticObstacles_unstructured():
    Nobs = globalvar.Nobs 

    planning_scale_ = globalvar.planning_scale_  
    vehicle_TPBV_ = globalvar.vehicle_TPBV_  
    vehicle_geometrics_ = globalvar.vehicle_geometrics_ 
    margin_obs_ = globalvar.margin_obs_ 
    #
    V_initial = CreateVehiclePolygon(vehicle_TPBV_.x0,vehicle_TPBV_.y0,vehicle_TPBV_.theta0)
    V_terminal = CreateVehiclePolygon(vehicle_TPBV_.xtf,vehicle_TPBV_.ytf,vehicle_TPBV_.thetatf)
    count = 0
    obj = np.array([])
    obstacle_cell = cell(1,Nobs)
    lx = planning_scale_.xmin
    ux = planning_scale_.xmax
    ly = planning_scale_.ymin
    uy = planning_scale_.ymax
    W = vehicle_geometrics_.vehicle_width
    L = vehicle_geometrics_.vehicle_length
    while count < Nobs:

        x = (ux - lx) * rand() + lx
        y = (uy - ly) * rand() + ly
        theta = 2 * math.pi * rand() - math.pi
        xru = x + L * rand() * np.cos(theta)
        yru = y + L * rand() * np.sin(theta)
        xrd = xru + W * rand() * np.sin(theta)
        yrd = yru - W * rand() * np.cos(theta)
        xld = x + W * rand() * np.sin(theta)
        yld = y - W * rand() * np.cos(theta)
        if xru < lx or xru > ux or xrd < lx or xrd > ux or xld < lx or xld > ux:
            continue
        elif yru < ly or yru > uy or yrd < ly or yrd > uy or yld < ly or yld > uy:
            continue
        temp_obj = np.array([[x,xru,xrd,xld],[y,yru,yrd,yld]])
        # check the initial/terminal point in the obstacle
        xv = np.array([x - margin_obs_,xru + margin_obs_,xrd + margin_obs_,xld - margin_obs_,x - margin_obs_])
        yv = np.array([y + margin_obs_,yru + margin_obs_,yrd - margin_obs_,yld - margin_obs_,y + margin_obs_])
        temp_obj_margin = np.array([[x - margin_obs_,xru + margin_obs_,xrd + margin_obs_,xld - margin_obs_,x - margin_obs_],[y + margin_obs_,yru + margin_obs_,yrd - margin_obs_,yld - margin_obs_,y + margin_obs_]])
        if inpolygon(vehicle_TPBV_.x0,vehicle_TPBV_.y0,xv,yv) > 0 or inpolygon(vehicle_TPBV_.xtf,vehicle_TPBV_.ytf,xv,yv) > 0:
            continue

        label_obj = 0
        # if the generated obstacle overlaps with other obstacles
        if count > 0:
            flag1 = checkObj_linev(temp_obj[:,0],temp_obj[:,1],obj)
            flag2 = checkObj_linev(temp_obj[:,3],temp_obj[:,2],obj)
            flag3 = checkObj_linev(temp_obj[:,0],temp_obj[:,2],obj)
            flag4 = checkObj_linev(temp_obj[:,1],temp_obj[:,3],obj)
            if flag1 + flag2 + flag3 + flag4 > 0:
                label_obj = 1
        # if the generated obstacle overlaps with initial and terminal states
        vehicle_state = np.array([V_initial.x[0:4],V_initial.y[0:4],V_terminal.x[0:4],V_terminal.y[0:4]])
        if count > 0:
            flag1 = checkObj_linev(temp_obj_margin[:,0],temp_obj_margin[:,1],vehicle_state)
            flag2 = checkObj_linev(temp_obj_margin[:,3],temp_obj_margin[:,2],vehicle_state)
            flag3 = checkObj_linev(temp_obj_margin[:,0],temp_obj_margin[:,2],vehicle_state)
            flag4 = checkObj_linev(temp_obj_margin[:,1],temp_obj_margin[:,3],vehicle_state)
            if flag1 + flag2 + flag3 + flag4 > 0:
                label_obj = 1
        # if the generated obstacle overlaps with other obstacles
        if label_obj > 0:
            continue
        # if the generated obstacle is effective
        
        #     current_obstacle.x = [x,xru,xrd,xld,x];
        #     current_obstacle.y = [y,yru,yrd,yld,y];
        current_obstacle = vclass()
        current_obstacle.x = np.array([x,xru,xrd,xld])
        current_obstacle.y = np.array([y,yru,yrd,yld])
        obstacle_cell[0][count] = current_obstacle
        count = count + 1
        #     current_obstacle.A = CalculatePolygonArea(current_obstacle);
        if count == 1:
            obj = temp_obj.copy()
        else:
            obj = np.hstack((obj,temp_obj))

    return obstacle_cell
#%%
def cell(rsize,csize):
    objcell = []
    for i in range(rsize):
        objcell.append([])
        for j in range(csize):
            objcell[i].append(0)
    return objcell

## transform the generated obstalces to 0-1 map
def costmap():
    mapset_ = globalvar.mapset_  
    map = np.zeros((round(mapset_.num_nodes_x), round(mapset_.num_nodes_y), 2))
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
        for x in range(minx, maxx):
            for y in range(miny, maxy):
                if(map[x][y][0] == 0 and inpolygon((x), (y), o_idx, o_idy)):
                    map[x][y][0] = 1
    for x in range(1, (round(astar_.num_nodes_x))):
        for y in range(1, (round(astar_.num_nodes_y))):
            map[x][y][1] = map[x-1][y][1]+map[x][y-1][1] + \
                map[x][y][0]-map[x-1][y-1][1]
    return map

def calc_xy_index(position, dim):
    if(dim == 0):
        pos = math.ceil(position / astar_.resolution_x)
    if(dim == 1):
        pos = math.ceil(position / astar_.resolution_y)
    return pos

# %%
import matplotlib.pyplot as plt
def VisualizeStaticResults(trajectory):
    obstacles_ = globalvar.obstacles_ 
    Nobs = globalvar.Nobs
    planning_scale_ = globalvar.planning_scale_
    nstep = len(trajectory.x)
    
    ## plot obstacle
    if Nobs > 0:
        for j in range(0,Nobs):
            vertex_x = obstacles_[0][j].x
            vertex_y = obstacles_[0][j].y
            plt.fill(vertex_x,vertex_y,color=(0.7451,0.7451,0.7451))
            # plt.hold(True)

    ## plot the planned trajectory
    plt.plot(trajectory.x,trajectory.y,'.-',color=(1,127 / 255,39 / 255),markersize=2,lw=1)
    # plt.hold(True)
    ## plot vehicle body
    for i in range(0,nstep):
        px = trajectory.x[i]
        py = trajectory.y[i]
        pth = trajectory.theta[i]
        V = CreateVehiclePolygon(px,py,pth)
        plt.plot(V.x,V.y,color=(153 / 255,217 / 255,234 / 255),lw=1)
        # plt.hold(True)

    ## plot start and terminal point
    plt.plot(trajectory.x[0],trajectory.y[0],'o',color=(1,201 / 255,14 / 255),lw=1)
    # plt.hold(True)
    plt.plot(trajectory.x[nstep-1],trajectory.y[nstep-1],'p',color=(1,201 / 255,14 / 255),lw=1)
    # plt.hold(True)
    plt.axis(np.array([planning_scale_.xmin,planning_scale_.xmax,planning_scale_.ymin,planning_scale_.ymax]))
    plt.axis('equal')
    plt.xlabel('x (m)',fontsize=12)
    plt.ylabel('y (m)',fontsize=12)
    # plt.hold(True)

    plt.show()
    return

# %%
import warnings

def VisualizeDynamicResults(trajectory):
    warnings.simplefilter("ignore")
    plt.ion()
    # warnings.warn('off')
    # hold('on')
    obstacles_ = globalvar.obstacles_
    Nobs = globalvar.Nobs
    planning_scale_ = globalvar.planning_scale_
    nstep = len(trajectory.x)
    plt.axis('equal')
    # set(gcf,'outerposition',get(0,'screensize'))
    for i in range(nstep):
        plt.cla()
        ## plot obstacle
        if Nobs > 0:
            for j in range(0,Nobs):
                vertex_x = obstacles_[0][j].x
                vertex_y = obstacles_[0][j].y
                plt.fill(vertex_x,vertex_y,color=(0.7451,0.7451,0.7451))
        
        plt.axis(np.array([planning_scale_.xmin,planning_scale_.xmax,planning_scale_.ymin,planning_scale_.ymax]))
        plt.axis('equal')

        # h1 = get(gca,'children')
        ## plot the planned trajectory
        plt.plot(trajectory.x,trajectory.y,'.-',Color=(1,127 / 255,39 / 255),markersize=2,lw = 1)
        ## plot vehicle body
        px = trajectory.x[i]
        py = trajectory.y[i]
        pth = trajectory.theta[i]
        V = CreateVehiclePolygon(px,py,pth)
        
        plt.plot(V.x,V.y,Color=(153 / 255,217 / 255,234 / 255),lw = 1)
        plt.pause(0.5)


    ## plot start and terminal point
    plt.plot(trajectory.x[0],trajectory.y[0],'o',Color=(1,201 / 255,14 / 255),lw = 1)
    plt.plot(trajectory.x[nstep-1],trajectory.y[nstep-1],'p',color=(1,201 / 255,14 / 255),lw = 1)
    plt.axis(np.array([planning_scale_.xmin,planning_scale_.xmax,planning_scale_.ymin,planning_scale_.ymax]))
    plt.axis('equal')
    plt.xlabel('x (m)',FontSize=12)
    plt.ylabel('y (m)',FontSize=12)
    plt.ioff()
    plt.show()
    return

# %%
if __name__ == "__main__":
    #%% genearate random static obstacle
    globalvar.obstacles_ = GenerateStaticObstacles_unstructured()

    global map
    costmap_ = costmap()

    ## please input your planned trajectory based on vehicle_TPBV_Set and costmap_

    VisualizeDynamicResults(trajectory)


