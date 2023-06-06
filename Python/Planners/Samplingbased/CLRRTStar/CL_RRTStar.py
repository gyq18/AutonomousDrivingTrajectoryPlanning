"""
Close Loop RRT* Planning Algorithm
"""

import numpy as np
import math
import globalvar
import matplotlib.pyplot as plt
import copy
from matplotlib.path import Path

L_max = 3 * (globalvar.vehicle_kinematics_.vehicle_v_max < 1.34) + 2.24 * globalvar.vehicle_kinematics_.vehicle_v_max * (globalvar.vehicle_kinematics_.vehicle_v_max >= 1.34 and globalvar.vehicle_kinematics_.vehicle_v_max < 5.36) + 12 * (globalvar.vehicle_kinematics_.vehicle_v_max >= 5.36)
        

class CL_RRTStar():
    """
    Class for CL_RRT* Planning
    """
    
    class Node():
        """
        CL_RRT* Node, stores the state of vehicle and the controller inputs of that state
        """
        def __init__(self, q, cmd, cost = 0) -> None:
            """
            Args:
                q (ndarray): [x, y, v, yaw(heading)]
                cmd (ndarray): [x, y] controller input
            """
            self.q = q
            self.cmd = cmd
            self.parent_idx = None
            self.cost = cost
            self.reachable = True
            self.terminate = False
        
    
    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 resolution_x = 0.5,
                 resolution_y = 0.5,
                 sim_timestep=0.1,
                 expand_dis=30.0,
                 goal_sample_rate=0.05,
                 connect_circle_dist=50.0,
                 loc_threshold = 0.5
                 ) -> None:
        """
        Args:
            start (ndarray): [x, y, v, yaw] initial position
            goal (ndarray): [x, y, v, yaw] goal position
            obstacle_list (list): list of obstacles
            resolution_x (float): resolution of costmap in x axis
            resolution_y (float): resolution of costmap in y axis 
            sim_timestep (float): simulation timestep (s)
            expand_dis (float, optional): _description_. Defaults to 30.0.
            goal_sample_rate (float, optional): probability of chosing goal as sample point. Defaults to 0.2.
            connect_circle_dist (float, optional):  Defaults to 50.0.
        """
        self.start = start
        self.goal = goal
        self.obstacle_list = obstacle_list
        self.resolution_x = resolution_x
        self.resolution_y = resolution_y
        self.sim_timestep = sim_timestep
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.connect_circle_dist = connect_circle_dist
        self.loc_threshold = loc_threshold
        self.grid_num_x = math.ceil(globalvar.planning_scale_.x_scale / self.resolution_x)
        self.grid_num_y = math.ceil(globalvar.planning_scale_.y_scale / self.resolution_y)
    
    def inpolygon(self, xq, yq, xv, yv):
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
    
    def CreateDilatedCostmap(self) -> None:
        """
        Generate Dilated Costmap
        """
        # tic = time.clock()
        xmin = globalvar.planning_scale_.xmin
        ymin = globalvar.planning_scale_.ymin
        
        self.costmap = np.zeros((self.grid_num_x, self.grid_num_y))

        length_unit = 0.5 * (self.resolution_x + self.resolution_y)
        dilate_radius = math.ceil(globalvar.vehicle_geometrics_.radius / length_unit)
        
        for i in range(globalvar.Nobs): 
            vx = self.obstacle_list[0][i].x
            vy = self.obstacle_list[0][i].y
            x_lb = min(vx)
            x_ub = max(vx)
            y_lb = min(vy)
            y_ub = max(vy)
            Nmin_x, Nmin_y = self.ConvertXYToIndex(x_lb,y_lb)
            Nmax_x, Nmax_y = self.ConvertXYToIndex(x_ub,y_ub)
            for j in range(Nmin_x, Nmax_x + 1):
                for k in range(Nmin_y, Nmax_y + 1):
                    if (self.costmap[j,k] == 1):
                        continue
                    cur_x = xmin + j * self.resolution_x
                    cur_y = ymin + k * self.resolution_y
                    if (self.inpolygon(cur_x, cur_y, self.obstacle_list[0][i].x, self.obstacle_list[0][i].y) > 0):
                        self.costmap[j,k] = 1
                        # dilate
                        for xx in range(max(0, j - dilate_radius), min(j + dilate_radius, self.grid_num_x)):
                            for yy in range(max(0, k - dilate_radius), min(k + dilate_radius, self.grid_num_y)):
                                if(self.costmap[xx, yy] == 1):
                                    continue
                                if (math.ceil(math.hypot(xx - j, yy - k)) < dilate_radius):
                                    self.costmap[xx, yy] = 1
                            
        # toc = time.clock()
        # print("consturct costmap in %.3f us" % ((toc - tic)*1000))
    
    def ConvertXYToIndex(self, x, y):
        """
        Conver location to index in costmap
        """
        idx1 = math.ceil((x - globalvar.planning_scale_.xmin) / self.resolution_x)
        idx2 = math.ceil((y - globalvar.planning_scale_.ymin) / self.resolution_y)
        if ((idx1 < self.grid_num_x) and (idx1 >= 0) and (idx2 < self.grid_num_y) and (idx2 >= 0)):
            return idx1, idx2
        if (idx1 >= self.grid_num_x):
            idx1 = self.grid_num_x - 1
        elif (idx1 < 0):
            idx1 = 0
        
        if (idx2 >= self.grid_num_y):
            idx2 = self.grid_num_y - 1
        elif (idx2 < 0):
            idx2 = 0
        return idx1, idx2
    
    def planning(self):
        """
        Planing
        """
        self.CreateDilatedCostmap()
        self.node_list = [self.Node(self.start, self.start[0:2])]
        # build the rrt-tree
        while(True):
            random_sample = self.get_random_sample()
            nearest_node_idx = self.find_nearest_node(random_sample)
            node_new_idx = self.create_new_node(nearest_node_idx, random_sample)
            if(node_new_idx is None):
                continue
            self.rewire(node_new_idx)
            if(self.is_goal(node_new_idx)):
                break
        
        # get traj
        self.traj = np.array(self.get_traj(node_new_idx)).T
        
        x = self.traj[0,:]
        y = self.traj[1,:]
        theta = self.traj[3,:]
        path_length = 0
        for i in range(x.shape[0] - 1):
            path_length += np.hypot(x[i+1] - x[i], y[i+1] - y[i]) 
             
        return x, y, theta, path_length, True
    
    def get_random_sample(self):
        """
        Get a random sample in controller input space
        """
        # tolerance for sample point to exceed planning scale (since sample point may not be reached considering the lookforward dis in pure_persuit)
        tolerance = 3
        if (np.random.uniform() < self.goal_sample_rate):
            return self.goal[0], self.goal[1]
        else:
            x = globalvar.planning_scale_.xmin - tolerance + (globalvar.planning_scale_.x_scale + 2 * tolerance) * np.random.uniform()
            y = globalvar.planning_scale_.ymin - tolerance + (globalvar.planning_scale_.y_scale + 2 * tolerance) * np.random.uniform()
            return x, y
        
    def create_new_node(self, node_parent_idx, random_sample):
        """
        Create node_new
        Args:
            random_sample (tuple): [x, y]
        """
        dx = random_sample[0] - self.node_list[node_parent_idx].cmd[0]
        dy = random_sample[1] - self.node_list[node_parent_idx].cmd[1]
        norm = math.hypot(dx, dy)
        if(norm == 0):
            return None
        dis_to_goal = math.hypot(self.node_list[node_parent_idx].q[0] - self.goal[0], self.node_list[node_parent_idx].q[1] - self.goal[1])
        _expend_dis = min(self.expand_dis, dis_to_goal)
        new_x = self.node_list[node_parent_idx].cmd[0] + dx * _expend_dis / norm
        new_y = self.node_list[node_parent_idx].cmd[1] + dy * _expend_dis / norm
        new_x = min(new_x, globalvar.planning_scale_.xmax)
        new_x = max(new_x, globalvar.planning_scale_.xmin)
        new_y = min(new_y, globalvar.planning_scale_.ymax)
        new_y = max(new_y, globalvar.planning_scale_.ymin)
             
        node_new = self.Node([0,0,0,0], np.array([new_x, new_y]))
        
        # decide whether or not the car should stop at the new point
        if(math.hypot(new_x - self.goal[0], new_y - self.goal[1]) <= 8):
            node_new.terminate = True

        feasible, traj, path_length = self.propagate(self.node_list[node_parent_idx], node_new)
        if (feasible):
            node_new.cost = self.node_list[node_parent_idx].cost + path_length
            node_new.parent_idx = node_parent_idx
            node_new.q = copy.deepcopy(traj[-1])
            self.node_list.append(node_new)
            return len(self.node_list) - 1
        else:
            return None
    
    def find_nearest_node(self, random_sample) -> int:
        """
        find the nearest reachable node to the random_sample given

        Args:
            random_sample(tuple): (x, y)
        
        Returns:
            int: index of the nearest node in self.node_list
        """
        dis = []
        reachable = []
        for node in self.node_list:
            dis.append(self.dubin_dis(node, random_sample))
            reachable.append(node.reachable)
        dis = np.array(dis)
        idx = np.where(reachable)[0]
        return idx[np.argmin(dis[idx])]
    
    def rewire(self, node_new_idx):
        """
        Rewire the given node and the nodes near it. If sucess, append the node to the tree
        """
        near_node_idx = self.find_near_nodes(node_new_idx)
        is_reparent = False
        # find a new parent for node_new
        for idx in near_node_idx:
            if (self.node_list[idx].reachable):
                dis = self.dubin_dis(self.node_list[idx], self.node_list[node_new_idx].q)
                if (dis + self.node_list[idx].cost < self.node_list[node_new_idx].cost):
                    # check feasibility
                    feasible , traj, path_length = self.propagate(self.node_list[idx], self.node_list[node_new_idx])
                    if (feasible):
                        self.node_list[node_new_idx].parent_idx = idx
                        self.node_list[node_new_idx].cost = path_length + self.node_list[idx].cost
                        self.node_list[node_new_idx].q = copy.deepcopy(traj[-1])
            else:
                continue
            
        # rewire other nodes
        for idx in near_node_idx:
            dis = self.dubin_dis(self.node_list[node_new_idx], self.node_list[idx].q)
            if(dis + self.node_list[node_new_idx].cost < self.node_list[idx].cost):
                feasible, traj, path_length = self.propagate(self.node_list[node_new_idx], self.node_list[idx])
                if(feasible):
                    self.node_list[idx].parent_idx = node_new_idx
                    self.node_list[idx].cost = dis + self.node_list[node_new_idx].cost
                    self.node_list[idx].q = copy.deepcopy(traj[-1])
                    is_reparent = True
        # update the tree
        if (is_reparent):
            self.update_tree(node_new_idx)
                          
    def is_goal(self, node_idx) -> bool:
        dx = self.goal[0] - self.node_list[node_idx].q[0]
        dy = self.goal[1] - self.node_list[node_idx].q[1]
        if math.hypot(dx, dy) <= self.loc_threshold and self.node_list[node_idx].terminate:
            return (True and self.node_list[node_idx].reachable)
        return False
        
    def propagate(self, node_from, node_to):
        """
        Propagate form node_from to node_to, using the control law describe in KuwataGNC08
        """
        t_min = 0.5
        # form the controller input(draw a line)
        c_x = np.linspace(node_from.cmd[0], node_to.cmd[0], num = 50)
        c_y = np.linspace(node_from.cmd[1], node_to.cmd[1], num = 50)
        # get the initial look-ahead point
        idx = self.get_look_ahead_point_idx(c_x, c_y, node_from.q)
        # generate v_profile
        ## params for calculating v_coast
        a_accel = abs(globalvar.vehicle_kinematics_.vehicle_a_max)
        a_decel = abs(globalvar.vehicle_kinematics_.vehicle_a_min)
        alpha_2 = -0.0252
        alpha_1 = 1.2344
        alpha_0 = -0.5347
        ## calculate v_coast
        L_1 = 3 * (node_from.q[2] < 1.34) + 2.24 * node_from.q[2] * (node_from.q[2] >= 1.34 and node_from.q[2] < 5.36) + 12 * (node_from.q[2] >= 5.36)
        if(node_to.terminate):
            L_min = 3
        else:
            L_min = L_max
        
        D = L_1 + math.hypot(c_x[-1] - c_x[idx], c_y[-1] - c_x[idx]) - L_min
        f = lambda v: v**2 / (2 * a_decel) + alpha_2 * v**2 + alpha_1 * v + alpha_0
        pi_to_pi = lambda angle: (angle + math.pi) % (2 * math.pi) - math.pi
        
        if(node_to.terminate):
            if D > (globalvar.vehicle_kinematics_.vehicle_v_max**2 - node_from.q[2]**2) / (2 * globalvar.vehicle_kinematics_.vehicle_a_max) + globalvar.vehicle_kinematics_.vehicle_v_max * t_min + f(globalvar.vehicle_kinematics_.vehicle_v_max):
                v_coast = globalvar.vehicle_kinematics_.vehicle_v_max
            else:
                A = 1 / (2*a_accel) + 1 / (2*a_decel) + alpha_2
                B = t_min + alpha_1
                C = alpha_0 - D - node_from.q[2]**2 / (2*a_accel)
                v_coast = (-1 * B + math.sqrt(B**2 - 4 * A * C)) / (2 * A)
               
            v_profile = []
            v_t = node_from.q[2]
            v_profile.append(v_t)
            state = 0 ## 0:ramp up; 1:coasting; 2:ramp down
            
            while(True):
                if(state == 0):
                    v_t = min(v_coast, v_t + a_accel * self.sim_timestep)
                    if (v_t == v_coast):
                        state = 1
                elif (state == 1):
                    v_t = v_coast
                    t_min -= self.sim_timestep
                    if (t_min <= 0):
                        state = 2
                elif (state == 2):
                    v_t = max(0, v_t - a_decel * self.sim_timestep)
                    if (v_t == 0):
                        v_profile.append(v_t)
                        break
                v_profile.append(v_t)
        else:
            v_coast = globalvar.vehicle_kinematics_.vehicle_v_max
            
            v_profile = []
            v_t = node_from.q[2]
            v_profile.append(v_t)
            state = 0 ## 0:ramp up; 1:coasting;
            while(D >= 0):
                if(state == 0):
                    v_t = min(v_coast, v_t + a_accel * self.sim_timestep)
                    D -= v_t * self.sim_timestep
                    if (v_t == v_coast):
                        state = 1
                elif (state == 1):
                    v_t = v_coast
                    D -= v_t * self.sim_timestep
                v_profile.append(v_t)                
        
        q_t = copy.deepcopy(node_from.q) ## temporary state
        
        traject = []
        traject.append(copy.deepcopy(q_t))
        idx = self.get_look_ahead_point_idx(c_x, c_y, q_t)
        ## steering control
        Lf = 3
        eta = q_t[3] - math.atan2((c_y[idx] - q_t[1]), (c_x[idx] - q_t[0]))
        eta = pi_to_pi(eta)
        
        if(abs(eta) < math.pi / 2):
            move_dir = 1 # forward
            delta = -math.atan2(2.0 * globalvar.vehicle_geometrics_.vehicle_length * math.sin(eta) / Lf, 1.0)
        else:
            move_dir = -1 # backward
            delta = -math.atan2(2.0 * globalvar.vehicle_geometrics_.vehicle_length * math.sin(math.pi - eta) / Lf, 1.0)
        path_length = 0
        
        
        for v in v_profile:
            if v == 0:
                continue
            idx = self.get_look_ahead_point_idx(c_x, c_y, q_t)
            ## steering control
            Lf = 3 * (v < 1.34) + 2.24 * v * (v >= 1.34 and v < 5.36) + 12 * (v >= 5.36)
            eta = q_t[3] - math.atan2((c_y[idx] - q_t[1]), (c_x[idx] - q_t[0]))
            eta = pi_to_pi(eta)
            
            if(abs(eta) < math.pi / 2):
                delta = -math.atan2(2.0 * globalvar.vehicle_geometrics_.vehicle_length * math.sin(eta) / Lf, 1.0)
            else:
                delta = -math.atan2(2.0 * globalvar.vehicle_geometrics_.vehicle_length * math.sin(math.pi - eta) / Lf, 1.0)
                                                                                                                                                                                                                                                                                                                                                                                                                                           
            if (delta > globalvar.vehicle_kinematics_.vehicle_phi_max):
                delta = globalvar.vehicle_kinematics_.vehicle_phi_max
            if (delta < globalvar.vehicle_kinematics_.vehicle_phi_min):
                delta = globalvar.vehicle_kinematics_.vehicle_phi_min
            
            ## update state
            path_length = path_length + v * self.sim_timestep
            q_t[0] += v * self.sim_timestep * math.cos(q_t[3]) * move_dir
            q_t[1] += v * self.sim_timestep * math.sin(q_t[3]) * move_dir
            q_t[2] =  (v * move_dir)
            q_t[3] += (v * move_dir) / globalvar.vehicle_geometrics_.vehicle_length * math.tan(delta)
            q_t[3] = pi_to_pi(q_t[3])
            ## check feasibility
            if(q_t[0] < globalvar.planning_scale_.xmin or q_t[0] > globalvar.planning_scale_.xmax \
                or q_t[1] < globalvar.planning_scale_.ymin or q_t[1] > globalvar.planning_scale_.ymax):
                
                return False, traject, path_length
            if(not self.check_collision(q_t)):
                return False, traject, path_length
            traject.append(copy.deepcopy(q_t))
        
        return True, traject, path_length
            
    def get_look_ahead_point_idx(self, c_x, c_y, q):
        Lf = 3 * (q[2] < 1.34) + 2.24 * q[2] * (q[2] >= 1.34 and q[2] < 5.36) + 12 * (q[2] >= 5.36)
        dx = [q[0] - icx for icx in c_x]
        dy = [q[1] - icy for icy in c_y]
        d = np.hypot(dx, dy)
        i = np.argmin(d)
        while(i < d.shape[0] - 1 and d[i] < Lf):
            i += 1
        return i
    
    def check_collision(self, q):
        idx_x, idx_y = self.ConvertXYToIndex(q[0], q[1])
        return self.costmap[idx_x, idx_y] == 0
    
    def find_near_nodes(self, node_new_idx):
        near_node_idx = []
        for idx, node in enumerate(self.node_list):
            if (self.dubin_dis(node, self.node_list[node_new_idx].q) < self.connect_circle_dist):
                near_node_idx.append(idx)
        return near_node_idx
    
    def find_goal_node_idx(self):
        min_dis = 10000
        min_idx = -1
        for idx, node in enumerate(self.node_list):
            if node.reachable:
                dx = node.q[0] - self.goal[0]
                dy = node.q[1] - self.goal[1]
                dis = math.hypot(dx, dy)
                if(dis <= self.loc_threshold):
                    return idx
                elif dis < min_dis:
                    min_dis = dis
                    min_idx = idx
        return min_idx

    def get_traj(self, goal_node_idx):
        path = [goal_node_idx]
        parent = self.node_list[goal_node_idx].parent_idx
        while(not parent is None):
            path.append(parent)
            parent = self.node_list[parent].parent_idx
        path.reverse()
        
        traj = [self.start]
        for i in range(len(path) - 1):
            flag, traj_temp, path_length = self.propagate(self.node_list[path[i]], self.node_list[path[i + 1]])
            traj = traj + traj_temp[1:]
        return traj
    
    def update_tree(self, parent_idx):
        for idx in range(len(self.node_list)):
            if (self.node_list[idx].parent_idx == parent_idx):
                if(self.node_list[parent_idx].reachable):
                    feasable, traj, path_length = self.propagate(self.node_list[parent_idx], self.node_list[idx])
                    if(feasable):
                        self.node_list[idx].cost = path_length + self.node_list[parent_idx].cost
                        self.node_list[idx].q = traj[-1]
                        self.node_list[idx].reachable = True
                    else:
                        # denote node as unreachable
                        self.node_list[idx].reachable = False
                else:
                    self.node_list[idx].reachable = False
                self.update_tree(idx)
    
    def dubin_dis(self, node_from, cmd):
        """
        Calculate dubins distance between two nodes

        Returns:
            float: dubins distance
        """
        x = node_from.q[0]
        y = node_from.q[1]
        theta = node_from.q[3]
        rho = globalvar.vehicle_kinematics_.min_turning_radius
        #now transform s.t. node is at (0,0,0) - RTMP 1109
        sxx = cmd[0] - x
        syy = cmd[1] - y
        sx_rel = sxx * math.cos(theta) - syy * math.sin(-theta)
        sy_rel = abs(sxx * math.sin(-theta) + syy * math.cos(theta))
        if  math.sqrt((sx_rel - 0)**2 + (sy_rel - rho)**2) < rho: #relative point is in Dp  - RTMP 1109
            df = math.sqrt((sx_rel**2) + (sy_rel + rho)**2)
            dc = math.sqrt((sx_rel**2) + (sy_rel - rho)**2)
            theta_i = math.atan2(sx_rel, rho - sy_rel)
            theta_c = (theta_i + 2*math.pi) % 2*math.pi
            phi = math.acos((5*rho**2 - df**2)/(4*rho**2))
            alpha = 2*math.pi - phi
            dis = rho*(alpha + math.asin(dc*math.sin(theta_c)/df) + math.asin((rho*math.sin(phi))/df))
        else:
            dc = math.sqrt((sx_rel**2) + (sy_rel - rho)**2)
            theta_i = math.atan2(sx_rel, rho - sy_rel)
            theta_c = (theta_i + 2*math.pi) % 2*math.pi
            dis = math.sqrt(dc**2 - rho**2) + rho*(theta_c - math.acos(rho/dc))
        return dis
        
    