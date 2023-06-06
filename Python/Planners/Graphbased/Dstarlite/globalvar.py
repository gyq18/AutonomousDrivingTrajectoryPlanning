import math
import numpy as np
class vehicle_geometrics_Set:
    def __init__(self) -> None:
        self.vehicle_wheelbase = 2.8  # L_W,wheelbase of the ego vehicle (m)
        self.vehicle_front_hang = 0.96 # L_F,front hang length of the ego vehicle (m)
        self.vehicle_rear_hang = 0.929 # L_R,rear hang length of the ego vehicle (m)
        self.vehicle_width = 1.942 # width of the ego vehicle (m)
        self.vehicle_length = self.vehicle_wheelbase + self.vehicle_front_hang + self.vehicle_rear_hang # length of the ego vehicle (m)
        self.radius = math.hypot(0.25 * self.vehicle_length, 0.5 * self.vehicle_width)
        self.r2x = 0.25 * self.vehicle_length - self.vehicle_rear_hang
        self.f2x = 0.75 * self.vehicle_length - self.vehicle_rear_hang


# vehicle kinematics settings
class vehicle_kinematics_Set:
    def __init__(self) -> None:
        self.vehicle_v_max = 2.5
        self.vehicle_v_min = -2.5 # upper and lower bounds of v(t) (m/s)
        self.vehicle_a_max = 0.5
        self.vehicle_a_min = -0.5 # upper and lower bounds of a(t) (m/s^2)
        self.vehicle_jerk_max = 0.5
        self.vehicle_jerk_min = -0.5 # upper and lower bounds of jerk(t) (m/s^3) 
        self.vehicle_phi_max = 0.7
        self.vehicle_phi_min = -0.7 # upper and lower bounds of phi(t) (rad)
        self.vehicle_omega_max = 0.5
        self.vehicle_omega_min = -0.5 # upper and lower bounds of omega(t) (rad/s)

        global vehicle_geometrics_ 
        self.min_turning_radius = vehicle_geometrics_.vehicle_wheelbase/math.tan(self.vehicle_phi_max)

#%% scenario settings
class planning_scale_Set:
    def __init__(self) -> None:
        self.xmin=0
        self.xmax=40
        self.ymin=0
        self.ymax=40  # space is a rectange, [lx,ux],[ly,uy]
        self.x_scale = self.xmax - self.xmin
        self.y_scale = self.ymax - self.ymin
        ## the followings are for discrete grid map generation, discretize the environment into a grid map of size m*n
        # self.m = 40
        # self_.n = 40

#%%
class hybrid_astar_Set:
    def __init__(self) -> None:
        self.resolution_x = 0.3
        self.resolution_y = 0.3
        self.resolution_theta = 0.5
        self.num_nodes_x = np.ceil(planning_scale_.x_scale / self.resolution_x) + 1
        self.num_nodes_y = np.ceil(planning_scale_.y_scale / self.resolution_x) + 1
        self.num_nodes_theta = np.ceil(2 * math.pi / self.resolution_theta) + 1
        self.penalty_for_backward = 1
        self.penalty_for_direction_changes = 3
        self.penalty_for_steering_changes = 0
        self.multiplier_H = 5.0
        self.multiplier_H_for_A_star = 2.0
        self.max_iter = 500
        self.max_time = 5
        self.simulation_step = 0.7
        self.Nrs = 10

#%% vehicle initial and terminal states settings
class vehicle_TPBV_Set:
    def __init__(self) -> None:
        self.x0=4
        self.y0=4
        self.theta0=0
        self.v0=0
        self.phi0=0
        self.a=0
        self.omega0=0

        self.xtf=32
        self.ytf=32
        self.thetatf=0
        self.vtf=0
        self.phitf=0
        self.a=0
        self.omegatf=0

#%% calculate of the vehicle edge position based on the position of the rear axle center
class vclass:
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.A = 0

global vehicle_geometrics_ 
vehicle_geometrics_ = vehicle_geometrics_Set()

global vehicle_kinematics_
vehicle_kinematics_ = vehicle_kinematics_Set()

global planning_scale_
planning_scale_ = planning_scale_Set()

global num_nodes_s # Nfe the step of trajectories
num_nodes_s=60

global vehicle_TPBV_
vehicle_TPBV_ = vehicle_TPBV_Set()

global obstacles_ 
global Nobs 
global margin_obs_ # margin_obs_ for dilated obstacles
margin_obs_=0.5
Nobs = 2

global costmap_