% 测试learning学到的决策动作是否合理
clc;clear; close all; clear global var;

%% vehicle settings
% vehicle geometrics settings
global vehicle_geometrics_ 
vehicle_geometrics_.vehicle_wheelbase = 2.8;  % L_W,wheelbase of the ego vehicle (m)
vehicle_geometrics_.vehicle_front_hang = 0.96; % L_F,front hang length of the ego vehicle (m)
vehicle_geometrics_.vehicle_rear_hang = 0.929; % L_R,rear hang length of the ego vehicle (m)
vehicle_geometrics_.vehicle_width = 1.942; % width of the ego vehicle (m)
vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang; % length of the ego vehicle (m)
vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.vehicle_length, 0.5 * vehicle_geometrics_.vehicle_width);
vehicle_geometrics_.r2x = 0.25 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
vehicle_geometrics_.f2x = 0.75 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
% vehicle kinematics settings
global vehicle_kinematics_ 
vehicle_kinematics_.vehicle_v_max = 2.5; vehicle_kinematics_.vehicle_v_min = -2.5; % upper and lower bounds of v(t) (m/s)
vehicle_kinematics_.vehicle_a_max = 0.5; vehicle_kinematics_.vehicle_a_min = -0.5; % upper and lower bounds of a(t) (m/s^2)
vehicle_kinematics_.vehicle_jerk_max = 0.5; vehicle_kinematics_.vehicle_jerk_min = -0.5; % upper and lower bounds of jerk(t) (m/s^3) 
vehicle_kinematics_.vehicle_phi_max = 0.7; vehicle_kinematics_.vehicle_phi_min = -0.7; % upper and lower bounds of phi(t) (rad)
vehicle_kinematics_.vehicle_omega_max = 0.5; vehicle_kinematics_.vehicle_omega_min = -0.5; % upper and lower bounds of omega(t) (rad/s)
vehicle_kinematics_.min_turning_radius = vehicle_geometrics_.vehicle_wheelbase/tan(vehicle_kinematics_.vehicle_phi_max);

%% scenario settings
global planning_scale_
planning_scale_.xmin=0; planning_scale_.xmax=60;planning_scale_.ymin=0;planning_scale_.ymax=60; %space is a rectange, [lx,ux],[ly,uy]
planning_scale_.x_scale = planning_scale_.xmax - planning_scale_.xmin;
planning_scale_.y_scale = planning_scale_.ymax - planning_scale_.ymin;

%% vehicle initial and terminal states settings
global vehicle_TPBV_
vehicle_TPBV_.x0=4; vehicle_TPBV_.y0=4;vehicle_TPBV_.theta0=0;
vehicle_TPBV_.v0=0;vehicle_TPBV_.phi0=0;vehicle_TPBV_.a0=0;vehicle_TPBV_.omega0=0;
% vehicle_TPBV_.xtf=52; vehicle_TPBV_.ytf=52;vehicle_TPBV_.thetatf=0;
vehicle_TPBV_.vtf=0;vehicle_TPBV_.phitf=0;vehicle_TPBV_.atf=0;vehicle_TPBV_.omegatf=0;

%% genearate random static obstacle
global margin_obs_ % margin_obs_ for dilated obstacles
margin_obs_=2;

%% For example, Hybrid Astar
global hybrid_astar_
hybrid_astar_.resolution_x = 0.3;
hybrid_astar_.resolution_y = 0.3;
hybrid_astar_.resolution_theta = 0.5;
hybrid_astar_.num_nodes_x = ceil(planning_scale_.x_scale / hybrid_astar_.resolution_x) + 1;
hybrid_astar_.num_nodes_y = ceil(planning_scale_.y_scale / hybrid_astar_.resolution_x) + 1;
hybrid_astar_.num_nodes_theta = ceil(2 * pi / hybrid_astar_.resolution_theta) + 1;
hybrid_astar_.penalty_for_backward = 1;
hybrid_astar_.penalty_for_direction_changes = 3;
hybrid_astar_.penalty_for_steering_changes = 0;
hybrid_astar_.multiplier_H = 5.0;
hybrid_astar_.multiplier_H_for_A_star = 2.0;
hybrid_astar_.max_iter = 500;
hybrid_astar_.max_time = 5;
hybrid_astar_.simulation_step = 0.7;
hybrid_astar_.Nrs = 10;

global num_nodes_s %Nfe the step of trajectories
num_nodes_s=60;
% global params_
collision_choice = 2;
transmethod_flag=1; %choose different path to trajectory method

% addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Graphbased');
pathfile=[pwd, '\npy-matlab'];
addpath(pathfile);
out_traj = readNPY('out_traj\0.npy');

theta_r=[0,1/4,1/3,1/2,3/4,2/3,1];

for id=98
    %% 原始训练数据集
    obsfile = [pwd, '\dataset-lisan\plot\obs\', num2str(id), '.csv'];
    taskfile = [pwd, '\dataset-new\dataset\task\', num2str(id), '.csv'];
    trajfile = [pwd, '\dataset-lisan\plot\traj\', num2str(id), '.csv'];
    obs = readmatrix(obsfile);
    traj_train = readmatrix(trajfile);
    trajectory.x=traj_train(:,1); trajectory.y=traj_train(:,2); trajectory.theta=traj_train(:,3); 
    nstep = length(trajectory.x);
    dist = zeros(nstep,1);
    phi_real = zeros(nstep,1);
    for ii = 1 :(nstep-1)
        dist(ii) = hypot(trajectory.x(ii+1)-trajectory.x(ii), trajectory.y(ii+1)-trajectory.y(ii));
        phi_real(ii) = (trajectory.theta(ii+1)-trajectory.theta(ii))*vehicle_geometrics_.vehicle_wheelbase/dist(ii);
    end

    phi_a = traj_train(:,4);
    atrajectory.x(1)=4; atrajectory.y(1)=4; atrajectory.theta(1)=0;
    for k = 2:length(phi_a)
        atrajectory.theta(k)=atrajectory.theta(k-1)+dist(k-1)*(tan(phi_a(k-1))/vehicle_geometrics_.vehicle_wheelbase);
        atrajectory.x(k) = atrajectory.x(k-1)+dist(k-1)*cos(atrajectory.theta(k-1));
        atrajectory.y(k) = atrajectory.y(k-1)+dist(k-1)*sin(atrajectory.theta(k-1));
    end

    %% 训练结果（需对训练结果进行处理）
    outtrajfile = [pwd, '\out_traj\', num2str(id-1), '.npy'];
    out_traj = readNPY(outtrajfile);
    [~,action] = max(out_traj,[],2); 
    phi_b = (action-1-7)/10; %从1-15转为-0.7到0.7
    phi_b = [0;phi_b];
    btrajectory.x(1)=4; btrajectory.y(1)=4; btrajectory.theta(1)=0;
    for k = 2:length(phi_b)
        btrajectory.theta(k)=btrajectory.theta(k-1)+dist(k-1)*(tan(phi_b(k-1))/vehicle_geometrics_.vehicle_wheelbase);
        btrajectory.x(k) = btrajectory.x(k-1)+dist(k-1)*cos(btrajectory.theta(k-1));
        btrajectory.y(k) = btrajectory.y(k-1)+dist(k-1)*sin(btrajectory.theta(k-1));
    end
    
    VisualizeStaticResults(trajectory,obs);
    VisualizeStaticResults(atrajectory,obs);
    VisualizeStaticResults(btrajectory,obs);

    if id>2000
        break;
    end
end

%% visualization for static results
function VisualizeStaticResults(trajectory,obs)
global planning_scale_
Nobs = size(obs,1);
nstep = length(trajectory.x);
figure;
%% plot obstacle
if Nobs>0
    for j=1:Nobs 
        vertex_x = obs(j,1:4);
        vertex_y = obs(j,5:8);
        fill(vertex_x,vertex_y,[0.7451 0.7451 0.7451]);
        hold on;
    end
end
%% plot the planned trajectory
plot(trajectory.x,trajectory.y,'.-','Color',[1 127/255 39/255],'LineWidth',1); 
hold on;
%% plot vehicle body
for i= 1:nstep
    px = trajectory.x(i);
    py = trajectory.y(i);
    pth = trajectory.theta(i);
    V = CreateVehiclePolygon(px,py,pth);
    plot(V.x, V.y,'Color',[153/255 217/255 234/255],'LineWidth',1); 
    hold on;
end
%% plot start and terminal point
plot(trajectory.x(1),trajectory.y(1),'o','Color',[1 201/255 14/255],'LineWidth',1);
hold on;
plot(trajectory.x(nstep),trajectory.y(nstep),'p','Color',[1 201/255 14/255],'LineWidth',1);
hold on;
axis([planning_scale_.xmin,planning_scale_.xmax,planning_scale_.ymin,planning_scale_.ymax]);
axis equal;
xlabel('x (m)','FontSize',12);
ylabel('y (m)','FontSize',12);
hold off;
end

%% calculate of the vehicle edge position based on the position of the rear axle center
function V = CreateVehiclePolygon(x,y,theta)
global vehicle_geometrics_
cos_theta = cos( theta );
sin_theta = sin( theta );
vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;
AX = x + ( vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase ) * cos_theta - vehicle_half_width * sin_theta;
BX = x + ( vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase ) * cos_theta + vehicle_half_width * sin_theta;
CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
AY = y + ( vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase ) * sin_theta + vehicle_half_width * cos_theta;
BY = y + ( vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase ) * sin_theta - vehicle_half_width * cos_theta;
CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
V.x = [ AX, BX, CX, DX, AX ];
V.y = [ AY, BY, CY, DY, AY ];
end