from types import SimpleNamespace
import numpy as np

config = SimpleNamespace()

# vehicle settings
# vehicle geometrics settings
vehicle_geometrics_ = SimpleNamespace()
vehicle_geometrics_.vehicle_wheelbase = 2.8  # L_W,wheelbase of the ego vehicle (m)
vehicle_geometrics_.vehicle_front_hang = 0.96 # L_F,front hang length of the ego vehicle (m)
vehicle_geometrics_.vehicle_rear_hang = 0.929 # L_R,rear hang length of the ego vehicle (m)
vehicle_geometrics_.vehicle_width = 1.942 # width of the ego vehicle (m)
vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang; # length of the ego vehicle (m)
vehicle_geometrics_.radius = np.hypot(0.25 * vehicle_geometrics_.vehicle_length, 0.5 * vehicle_geometrics_.vehicle_width)
vehicle_geometrics_.r2x = 0.25 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang
vehicle_geometrics_.f2x = 0.75 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang
# vehicle kinematics settings
vehicle_kinematics_  = SimpleNamespace()
vehicle_kinematics_.vehicle_v_max = 2.5 
vehicle_kinematics_.vehicle_v_min = -2.5 # upper and lower bounds of v(t) (m/s)
vehicle_kinematics_.vehicle_a_max = 0.5 
vehicle_kinematics_.vehicle_a_min = -0.5 # upper and lower bounds of a(t) (m/s^2)
vehicle_kinematics_.vehicle_jerk_max = 0.5 
vehicle_kinematics_.vehicle_jerk_min = -0.5 # upper and lower bounds of jerk(t) (m/s^3) 
vehicle_kinematics_.vehicle_phi_max = 0.7
vehicle_kinematics_.vehicle_phi_min = -0.7 # upper and lower bounds of phi(t) (rad)
vehicle_kinematics_.vehicle_omega_max = 0.5 
vehicle_kinematics_.vehicle_omega_min = -0.5 # upper and lower bounds of omega(t) (rad/s)
vehicle_kinematics_.min_turning_radius = vehicle_geometrics_.vehicle_wheelbase/np.tan(vehicle_kinematics_.vehicle_phi_max)

## scenario settings
planning_scale_ = SimpleNamespace()
planning_scale_.xmin=0
planning_scale_.xmax=60
planning_scale_.ymin=0
planning_scale_.ymax=60; #space is a rectange, [lx,ux],[ly,uy]
planning_scale_.x_scale = planning_scale_.xmax - planning_scale_.xmin
planning_scale_.y_scale = planning_scale_.ymax - planning_scale_.ymin

## vehicle initial and terminal states settings
vehicle_TPBV_ = SimpleNamespace()
vehicle_TPBV_.x0=4
vehicle_TPBV_.y0=52
vehicle_TPBV_.theta0=0
vehicle_TPBV_.v0=0
vehicle_TPBV_.phi0=0
vehicle_TPBV_.a=0
vehicle_TPBV_.omega0=0
vehicle_TPBV_.xtf=52
vehicle_TPBV_.ytf=4
vehicle_TPBV_.thetatf=np.pi/2
vehicle_TPBV_.vtf=0
vehicle_TPBV_.phitf=0
vehicle_TPBV_.a=0
vehicle_TPBV_.omegatf=0


config.vehicle_geometrics_ = vehicle_geometrics_
config.vehicle_kinematics_ = vehicle_kinematics_
config.planning_scale_ = planning_scale_
config.vehicle_TPBV_ = vehicle_TPBV_


