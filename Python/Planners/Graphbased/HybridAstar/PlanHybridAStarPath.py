from cmath import  inf, pi
from math import ceil, hypot,cos,sin,tan
from pandas import isnull
from time import time
import globalvar
import numpy as np
from numpy.linalg import norm as norm
from numpy.random import randn
from reeds_shepp_pathplanning import reeds_shepp_path_planning

global planning_scale_
planning_scale_ = globalvar.planning_scale_
global hybrid_astar_
hybrid_astar_ = globalvar.hybrid_astar_
global vehicle_TPBV_
vehicle_TPBV_ = globalvar.vehicle_TPBV_
global vehicle_kinematics_
vehicle_kinematics_ = globalvar.vehicle_kinematics_
global vehicle_geometrics_
vehicle_geometrics_ = globalvar.vehicle_geometrics_


def PlanHybridAStarPath(costmap):
    grid_space_ = np.empty((int(hybrid_astar_.num_nodes_x),int(hybrid_astar_.num_nodes_y),int(hybrid_astar_.num_nodes_theta)),dtype=object)
    end_config = [vehicle_TPBV_.xtf,vehicle_TPBV_.ytf,vehicle_TPBV_.thetatf]
    start_config = [vehicle_TPBV_.x0,vehicle_TPBV_.y0,vehicle_TPBV_.theta0]
    start_config = np.array(start_config)
    end_config = np.array(end_config)
    goal_ind = Convert3DimConfigToIndex(end_config)
    init_node = np.zeros([1,16])
    init_node[0,0:3] = start_config
    init_node[0,5] = CalculateH(start_config,end_config,costmap)
    init_node[0,4] = 0
    init_node[0,3] = init_node[0,4] + hybrid_astar_.multiplier_H*init_node[0,5]
    init_node[0,6] = 1
    init_node[0,8:11] = Convert3DimConfigToIndex(start_config)
    init_node[0,11:14] = [-999,-999,-999]
    openlist_ = init_node

    grid_space_[int(init_node[0,8]),int(init_node[0,9]),int(init_node[0,10])] = init_node
    expansion_pattern = np.array([[1,-vehicle_kinematics_.vehicle_phi_max],[1,0],[1,vehicle_kinematics_.vehicle_phi_max],[-1,-vehicle_kinematics_.vehicle_phi_max],[-1,0],[-1,vehicle_kinematics_.vehicle_phi_max]])
    completeness_flag = 0
    complete_via_rs_flag = 0
    best_ever_val = inf
    best_ever_ind = init_node[0,8:11]
    path_length = 0
    iter = 0

    tic = time()
    while len(openlist_)>0 and iter <= hybrid_astar_.max_iter and completeness_flag==0 and time()-tic <= hybrid_astar_.max_time:
        iter+=1
        cur_node_order = np.argmin(openlist_[:,3])
        if isinstance(cur_node_order,list):
            cur_node_order = cur_node_order[len(cur_node_order)-1]       
        cur_node = openlist_[cur_node_order,:]
        cur_config = cur_node[0:3]
        cur_ind = cur_node[8:11]
        cur_g = cur_node[4]
        cur_v = cur_node[14]
        cur_phi = cur_node[15]
        if iter % hybrid_astar_.Nrs==0:
            [x_rs,y_rs,theta_rs,path_length] = GenerateRsPath(cur_config,end_config)
            config = np.hstack((x_rs,y_rs,theta_rs))
            if Is3DNodeValid(config,costmap):
                completeness_flag = 1
                complete_via_rs_flag = 1
                best_ever_ind = cur_ind
                break
        openlist_ = np.delete(openlist_,cur_node_order,axis=0)
        grid_space_[int(cur_ind[0]),int(cur_ind[1]),int(cur_ind[2])][0,6] = 0
        grid_space_[int(cur_ind[0]),int(cur_ind[1]),int(cur_ind[2])][0,7] = 1
        expansion_pattern = np.array(expansion_pattern)
        for i in range(6):
            child_node_v = expansion_pattern[i,0]
            child_node_phi = expansion_pattern[i,1]
            child_node_config = SimulateForUnitDistance(cur_config,child_node_v,child_node_phi,hybrid_astar_.simulation_step)
            child_node_ind = Convert3DimConfigToIndex(child_node_config)
            if np.all(grid_space_[int(child_node_ind[0]),int(child_node_ind[1]),int(child_node_ind[2])])!=None and grid_space_[int(child_node_ind[0]),int(child_node_ind[1]),int(child_node_ind[2])][0,7]==1:
                continue
            child_g = cur_g + hybrid_astar_.simulation_step + hybrid_astar_.penalty_for_direction_changes*abs(cur_v-child_node_v) + hybrid_astar_.penalty_for_steering_changes*abs(cur_phi-child_node_phi)
            if np.all(grid_space_[int(child_node_ind[0]),int(child_node_ind[1]),int(child_node_ind[2])])!=None:
                if grid_space_[int(child_node_ind[0]),int(child_node_ind[1]),int(child_node_ind[2])][0,4] > child_g + 0.1:
                    order = findIndex(openlist_,child_node_ind)
                    openlist_ = np.delete(openlist_,order,axis=0)
                    child_node_update = grid_space_[int(child_node_ind[0]),int(child_node_ind[1]),int(child_node_ind[2])]
                    child_node_update[0,4] = child_g
                    child_node_update[0,3] = child_node_update[0,4] + hybrid_astar_.multiplier_H*child_node_update[0,5]
                    child_node_update[0,11:14] = cur_ind
                    child_node_update[0,14:16] = [child_node_v,child_node_phi]
                    openlist_ = np.vstack((openlist_,child_node_update))
                    grid_space_[int(child_node_ind[0]),int(child_node_ind[1]),int(child_node_ind[2])] = child_node_update
                continue
            child_node = np.zeros([1,16])
            if Is3DNodeValid(child_node_config,costmap)==0:
                child_node[0,7] = 1
                grid_space_[int(child_node_ind[0]),int(child_node_ind[1]),int(child_node_ind[2])] = child_node
                continue
            child_node[0,0:3] = child_node_config
            child_node[0,4] = child_g
            child_node[0,5] = CalculateH(child_node_config,end_config,costmap)
            child_node[0,3] = child_node[0,4]+hybrid_astar_.multiplier_H*child_node[0,5]
            child_node[0,6] = 1
            child_node[0,8:11] = child_node_ind
            child_node[0,11:14] = cur_ind
            child_node[0,14:16] = [child_node_v,child_node_phi]
            openlist_ = np.vstack((openlist_,child_node))
            grid_space_[int(child_node_ind[0]),int(child_node_ind[1]),int(child_node_ind[2])] = child_node

            if child_node[0,5] < best_ever_val:
                best_ever_val = child_node[0,5]
                best_ever_ind = child_node_ind
            if np.any(np.array(child_node_ind)-np.array(goal_ind))==False:
                completeness_flag = 1
                best_ever_ind = goal_ind
                break
    cur_best_parent_ind = grid_space_[int(best_ever_ind[0]),int(best_ever_ind[1]),int(best_ever_ind[2])][0,11:14]
    x = grid_space_[int(best_ever_ind[0]),int(best_ever_ind[1]),int(best_ever_ind[2])][0,0]
    y = grid_space_[int(best_ever_ind[0]),int(best_ever_ind[1]),int(best_ever_ind[2])][0,1]
    theta = grid_space_[int(best_ever_ind[0]),int(best_ever_ind[1]),int(best_ever_ind[2])][0,2]

    while cur_best_parent_ind[0] > -1:
        path_length = path_length + hybrid_astar_.simulation_step
        cur_node = grid_space_[int(cur_best_parent_ind[0]),int(cur_best_parent_ind[1]),int(cur_best_parent_ind[2])]
        cur_best_parent_ind = cur_node[0,11:14]
        x = np.vstack((cur_node[0,0],x))
        y = np.vstack((cur_node[0,1],y))
        theta = np.vstack((cur_node[0,2],theta))
    
    if completeness_flag:
        if complete_via_rs_flag:
            x_rs = x_rs.T
            y_rs = y_rs.T
            theta_rs = theta_rs.T
            x = x.T[0]
            y = y.T[0]
            theta = theta.T[0]
            
            if x_rs.shape[1] > 1:
                x = np.hstack((x,x_rs[0,1:]))
                y = np.hstack((y,y_rs[0,1:]))
                theta = np.hstack((theta,theta_rs[0,1:]))
        else:
            x = np.hstack((x,end_config[0]))
            y = np.hstack((y,end_config[1]))
            theta = np.hstack((theta,end_config[2]))

        [x,y,theta] = ResamplePathWithEqualDistance(x,y,theta)
        path_length = 0
        for i in range(len(x)-1):
            path_length = path_length+hypot(x[i+1]-x[i],y[i+1]-y[i])
    return [x,y,theta,path_length,completeness_flag]

def ResamplePathWithEqualDistance(x,y,theta):
    for i in range(1,len(theta)):
        while(theta[i]-theta[i-1] > pi):
            theta[i] = theta[i] - 2*pi
        while(theta[i]-theta[i-1] < -pi):
            theta[i] = theta[1] + 2*pi
    distance = hypot(x[1]-x[0],y[1]-y[0])
    LARGE_NUM = round(distance*100)
    temp = np.linspace(x[0],x[1],LARGE_NUM)
    temp = temp[:LARGE_NUM-1]
    x_extend = temp
    temp = np.linspace(y[0],y[1],LARGE_NUM)
    temp = temp[:LARGE_NUM-1]
    y_extend = temp
    temp = np.linspace(theta[0],theta[1],LARGE_NUM)
    temp = temp[:LARGE_NUM-1]
    theta_extend = temp
    for i in range(1,len(x)-1):
        distance = hypot(x[i+1]-x[i],y[i+1]-y[i])
        LARGE_NUM = round(distance*100)
        temp = np.linspace(x[i],x[i+1],LARGE_NUM)
        temp = temp[:LARGE_NUM-1]
        x_extend = np.hstack((x_extend,temp))

        temp = np.linspace(y[i],y[i+1],LARGE_NUM)
        temp = temp[:LARGE_NUM-1]
        y_extend = np.hstack((y_extend,temp))
        temp = np.linspace(theta[i],theta[i+1],LARGE_NUM)
        temp = temp[:LARGE_NUM-1]
        theta_extend = np.hstack((theta_extend,temp))
    x_extend = np.hstack((x_extend,x[len(x)-1]))
    y_extend = np.hstack((y_extend,y[len(y)-1]))
    theta_extend = np.hstack((theta_extend,theta[len(theta)-1]))

    num_nodes_s = globalvar.num_nodes_s
    index = np.round(np.linspace(0,len(x_extend)-1,num_nodes_s))
    index = index.astype(int)
    x = x_extend[index]
    y = y_extend[index]
    theta = theta_extend[index]
    return [x,y,theta]

def findIndex(openlist,child_ind):
    for i in range(openlist.shape[0]):
        if openlist[i,8]==child_ind[0] and openlist[i,9]==child_ind[1] and openlist[i,10]==child_ind[2]:
            return i

def find2DIndex(openlist,child_ind):
    for i in range(openlist.shape[0]):
        if openlist[i,7]==child_ind[0] and openlist[i,8]==child_ind[1]:
            return i

def xy2ind(x,y):
    N = hybrid_astar_.num_nodes_y
    inx = np.ceil((x-planning_scale_.xmin)/hybrid_astar_.resolution_x)
    inx = inx.astype(int)
    iny = np.ceil((y-planning_scale_.ymin)/hybrid_astar_.resolution_y)
    iny = iny.astype(int)
    row = N-iny-1
    row = row.astype(int)
    col = inx
    if isinstance(row,int):
        if col <= hybrid_astar_.num_nodes_x-1 and row <= hybrid_astar_.num_nodes_y-1 and row >=0:
            return row,col
        if col > hybrid_astar_.num_nodes_x-1:
            col = int(hybrid_astar_.num_nodes_x)-1
        elif col < 0:
            col = 0
        if row > hybrid_astar_.num_nodes_y-1:
            row = int(hybrid_astar_.num_nodes_y)-1
        elif row < 0:
            row= 0  
    return row,col

def SimulateForUnitDistance(cur_config,v,phi,simulate_step):
    vehicle_geometrics_ = globalvar.vehicle_geometrics_
    Nfe = 10
    hi = simulate_step/Nfe
    x = cur_config[0]
    y = cur_config[1]
    theta = cur_config[2]
    for i in range(Nfe):
        x = cos(theta)*v*hi+x
        y = sin(theta)*v*hi+y
        theta = tan(phi)*v/vehicle_geometrics_.vehicle_wheelbase*hi+theta
        x = x.real
        y = y.real
        theta = theta.real
    child_node_config = [x,y,theta]
    return child_node_config

def CalculateH(start_config,end_config,costmap):
    dist = norm(start_config[0:2]-end_config[0:2])
    distance_nonholonomic_without_collision_avoidance = max(dist,CalculateRsPathLength(start_config,end_config))
    distance_holonomic_with_collision_avoidance = CalculateAstarPathLength(start_config,end_config,costmap)
    val = max(distance_nonholonomic_without_collision_avoidance,distance_holonomic_with_collision_avoidance)
    return val

def CalculateAstarPathLength(start_config,end_config,costmap):
    hybrid_astar = globalvar.hybrid_astar_
    begin_config = start_config[0:2]
    end_config = end_config[0:2]
    grid_space_2D_ = np.empty((int(hybrid_astar_.num_nodes_x),int(hybrid_astar_.num_nodes_y)),dtype=object)
    init_node = np.zeros((1,11))
    init_node[0,0:2] = begin_config
    init_node[0,3] = 0
    init_node[0,4] = abs(init_node[0,0:2]-end_config).sum()
    init_node[0,2] = init_node[0,3] + hybrid_astar.multiplier_H_for_A_star*init_node[0,4]+0.001*randn()
    init_node[0,5] = 1
    init_node[0,7:9] = Convert2DimConfigToIndex(begin_config)
    init_node[0,9:11] = [-999,-999]
    openlist_ = init_node
    goal_ind = Convert2DimConfigToIndex(end_config)
    grid_space_2D_[int(init_node[0,7]),int(init_node[0,8])] = init_node[0]
    expansion_pattern = np.array([[-1,1],[-1,0],[-1,-1],[0,1],[0,-1],[1,1],[1,0],[1,-1]])*hybrid_astar.resolution_x
    expansion_length = np.array([[1.414],[1],[1.414],[1],[1],[1.414],[1],[1.414]])*hybrid_astar.resolution_x
    iter = 0

    while len(openlist_) > 0 and iter <= pow(hybrid_astar.num_nodes_x,2):
        iter = iter+1
        cur_node_order = np.argmin(openlist_[:,2])
        if isinstance(cur_node_order,list):
            cur_node_order = cur_node_order[len(cur_node_order)-1] 
        cur_node = openlist_[cur_node_order,:]
        cur_config = cur_node[0:2]
        cur_ind = cur_node[7:9]
        cur_g = cur_node[3]

        openlist_ = np.delete(openlist_,cur_node_order,axis=0)
        grid_space_2D_[int(cur_ind[0]),int(cur_ind[1])][5] = 0
        grid_space_2D_[int(cur_ind[0]),int(cur_ind[1])][6] = 1
        for i in range(8):
            child_node_config = cur_config + expansion_pattern[i,:]
            child_node_ind = Convert2DimConfigToIndex(child_node_config)
            child_g = cur_g + expansion_length[i,0]
            child_h = abs(child_node_config - end_config).sum()
            child_f = child_g + hybrid_astar.multiplier_H_for_A_star*child_h
            child_node_prepare = np.hstack((child_node_config,child_f,child_g,child_h,1,0,child_node_ind,cur_ind))
            if np.all(isnull(grid_space_2D_[int(child_node_ind[0]),int(child_node_ind[1])]))==False:
                if grid_space_2D_[int(child_node_ind[0]),int(child_node_ind[1])][6]==1:
                    continue
                if grid_space_2D_[int(child_node_ind[0]),int(child_node_ind[1])][3] > child_g + 0.1:
                    order = find2DIndex(openlist_,child_node_ind)
                    openlist_ = np.delete(openlist_,order,axis=0)
                    grid_space_2D_[int(child_node_ind[0]),int(child_node_ind[1])] = child_node_prepare
                    openlist_ = np.vstack((openlist_,child_node_prepare))
            else:
                if Is2DNodeValid(child_node_config,child_node_ind,costmap):
                    openlist_ = np.vstack((openlist_,child_node_prepare))
                    grid_space_2D_[int(child_node_ind[0]),int(child_node_ind[1])] = child_node_prepare
                    if abs(np.array(child_node_ind)-np.array(goal_ind)).sum()==0:
                        path_length = child_g
                        return path_length
                else:
                    child_node_prepare[6] = 1
                    child_node_prepare[5] = 0
                    grid_space_2D_[int(child_node_ind[0]),int(child_node_ind[1])] = child_node_prepare
    path_length = abs(begin_config-end_config).sum()
    return path_length               

def CalculateRsPathLength(start_config,end_config):
    [x,y,theta,length]=GenerateRsPath(start_config,end_config)
    return length

def GenerateRsPath(startPose,goalPose):
    start_x = startPose[0]
    start_y = startPose[1]
    start_theta = startPose[2]
    goal_x = goalPose[0]
    goal_y = goalPose[1]
    goal_theta = goalPose[2]
    x_rs,y_rs,theta_rs,length = reeds_shepp_path_planning(start_x,start_y,start_theta,goal_x,goal_y,goal_theta,vehicle_kinematics_.min_turning_radius,hybrid_astar_.simulation_step)
    x_rs = np.array(x_rs).reshape(-1,1)
    y_rs = np.array(y_rs).reshape(-1,1)
    theta_rs = np.array(theta_rs).reshape(-1,1)
    return [x_rs,y_rs,theta_rs,length]

def Convert3DimConfigToIndex(config):
    global planning_scale_
    planning_scale_ = globalvar.planning_scale_
    ind1 = ceil((config[0]-planning_scale_.xmin)/hybrid_astar_.resolution_x) 
    ind2 = ceil((config[1]-planning_scale_.ymin)/hybrid_astar_.resolution_y) 
    ind3 = ceil((RegulateAngle(config[2]))/hybrid_astar_.resolution_theta)
    if ind1 <= hybrid_astar_.num_nodes_x-1 and ind1 >= 0 and ind2 <= hybrid_astar_.num_nodes_y-1 and ind2 >=0:
        return [ind1,ind2,ind3]   
    if ind1 > hybrid_astar_.num_nodes_x-1:
        ind1 = hybrid_astar_.num_nodes_x-1
    elif ind1 < 0:
        ind1 = 0
    if ind2 > hybrid_astar_.num_nodes_y-1:
        ind2 = hybrid_astar_.num_nodes_y-1
    elif ind2 < 0:
        ind2 = 0   
    return [ind1,ind2,ind3]

def Convert2DimConfigToIndex(config):
    ind1 = ceil((config[0]-planning_scale_.xmin)/hybrid_astar_.resolution_x) 
    ind2 = ceil((config[1]-planning_scale_.ymin)/hybrid_astar_.resolution_y) 
    if ind1 <= hybrid_astar_.num_nodes_x-1 and ind2 <= hybrid_astar_.num_nodes_y-1 and ind2 >=0:
        return [ind1,ind2] 
    if ind1 > hybrid_astar_.num_nodes_x-1:
        ind1 = hybrid_astar_.num_nodes_x-1
    elif ind1 < 0:
        ind1 = 0
    if ind2 > hybrid_astar_.num_nodes_y-1:
        ind2 = hybrid_astar_.num_nodes_y-1
    elif ind2 < 0:
        ind2 = 0   
    return [ind1,ind2]

def RegulateAngle(angle):
    while(angle > 2*pi+0.000001):
        angle = angle - 2*pi
    while(angle < -0.000001):
        angle = angle + 2*pi
    return angle

def check_map(map,r,c):
    l = len(r)
    for i in range(l):
        if map[r[i],c[i]]==1:
            return 0
    return 1

def Is3DNodeValid(child_node_config,costmap):
    child_node_config = np.array(child_node_config)
    child_node_config = child_node_config.reshape(-1,3)
    planning_scale_ = globalvar.planning_scale_
    hybrid_astar_ = globalvar.hybrid_astar_
    vehicle_geometrics = globalvar.vehicle_geometrics_
    is_collision_free = 0
    xr = child_node_config[:,0]+vehicle_geometrics.r2x*np.cos(child_node_config[:,2])
    xr = xr.reshape(-1,1)
    yr = child_node_config[:,1]+vehicle_geometrics.r2x*np.sin(child_node_config[:,2])
    yr = yr.reshape(-1,1)
    xf = child_node_config[:,0]+vehicle_geometrics.f2x*np.cos(child_node_config[:,2])
    xf = xf.reshape(-1,1)
    yf = child_node_config[:,1]+vehicle_geometrics.f2x*np.sin(child_node_config[:,2])
    yf = yf.reshape(-1,1)
    xx = np.vstack([xr,xf])
    xx = xx.astype(float)
    yy = np.vstack([yr,yf])
    yy = yy.astype(float)
    if (xx > planning_scale_.xmax-vehicle_geometrics.radius*1.01).sum():
        return is_collision_free
    elif (xx < planning_scale_.xmin+vehicle_geometrics.radius*1.01).sum():
        return is_collision_free
    elif (yy > planning_scale_.ymax-vehicle_geometrics.radius*1.01).sum():
        return is_collision_free
    elif (yy < planning_scale_.ymin+vehicle_geometrics.radius*1.01).sum():
        return is_collision_free
    indr,indc = xy2ind(xx,yy)
    m = check_map(costmap,indr,indc)
    if m==0:
        return is_collision_free
    else:
        is_collision_free=1
        return is_collision_free
    
def Is2DNodeValid(child_node_config,child_node_ind,costmap):
    N = hybrid_astar_.num_nodes_y
    is_collision_free = 1
    ind1 = int(N-child_node_ind[1]-1)
    ind2 = int(child_node_ind[0])
    if costmap[ind1,ind2]==1:
        is_collision_free = 0
        return is_collision_free
    if child_node_config[0] > planning_scale_.xmax or child_node_config[0] < planning_scale_.xmin or child_node_config[1] > planning_scale_.ymax or child_node_config[1] < planning_scale_.ymin:
        is_collision_free = 0
        return is_collision_free
    return is_collision_free
