%% 比较A*和神经网络作为优化的初始解的情况
clc;clear; close all; clear global var;clear global

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

pathfile=[pwd, '\npy-matlab'];
addpath(pathfile);
out_traj = readNPY('out_traj\0.npy');

theta_r=[0,1/4,1/3,1/2,3/4,2/3,1];

global obstacles_ Nobs 
Nobs = 10;

for id=28
    %% 原始训练数据集
    obsfile = [pwd, '\dataset-lisan\plot\obs\', num2str(id), '.csv'];
    taskfile = [pwd, '\dataset-lisan\plot\task\', num2str(id), '.csv'];
    trajfile = [pwd, '\dataset-lisan\plot\traj\', num2str(id), '.csv'];
    obs = readmatrix(obsfile);
    
    %% 将障碍物转为全局变量
    obstacles_ = cell(1, Nobs);
    for i =1:Nobs    
        current_obstacle.x = obs(i,1:4);
        current_obstacle.y = obs(i,5:8);
        obstacles_ {1,i}=current_obstacle;
    end
    
    %% 读出终点的位置和角度
    task = readmatrix(taskfile);
    vehicle_TPBV_.xtf=task(1,2); vehicle_TPBV_.ytf=task(2,2);vehicle_TPBV_.thetatf=task(3,2);
    
    
    %% 原始优化轨迹
    traj_train = readmatrix(trajfile);
    phi_a = traj_train(:,4);
    atrajectory.x(1)=4; atrajectory.y(1)=4; atrajectory.theta(1)=0;
    for k = 2:length(phi_a)
        atrajectory.theta(k)=atrajectory.theta(k-1) + (tan(phi_a(k-1))/vehicle_geometrics_.vehicle_wheelbase);
        atrajectory.x(k) = atrajectory.x(k-1) + cos(atrajectory.theta(k-1));
        atrajectory.y(k) = atrajectory.y(k-1) + sin(atrajectory.theta(k-1));
    end

    %% 神经网络训练结果
    % 初始解
    outtrajfile = [pwd, '\out_traj\', num2str(id-1), '.npy'];
    out_traj = readNPY(outtrajfile);
    [~,action] = max(out_traj,[],2); 
    phi_b = (action-1-7)/10; %从1-15转为-0.7到0.7
    phi_b = [0;phi_b];
    btrajectory.x(1)=4; btrajectory.y(1)=4; btrajectory.theta(1)=0;
    tic;
    for k = 2:length(phi_b)
        btrajectory.theta(k)=btrajectory.theta(k-1) + (tan(phi_b(k-1))/vehicle_geometrics_.vehicle_wheelbase);
        btrajectory.x(k) = btrajectory.x(k-1) + cos(btrajectory.theta(k-1));
        btrajectory.y(k) = btrajectory.y(k-1) + sin(btrajectory.theta(k-1));
    end
    btrajectory = PlanSpeedForStaticScenarios(btrajectory.x, btrajectory.y, btrajectory.theta,length(phi_b));
    % 优化解
    collision_choice = 2;
    [btrajectory2,isfeasible] = PlanL1SCPTrajectory(btrajectory,collision_choice);
    time_net=toc;
    
    %% Astar的结果
    tic;
    [x, y, theta, path_length, completeness_flag] = PlanAStarPath();
    ctrajectory = PlanSpeedForStaticScenarios(x, y, theta, path_length);
    [ctrajectory2,isfeasible] = PlanL1SCPTrajectory(ctrajectory,collision_choice);
    time_a = toc;
    
    VisualizeStaticResults(atrajectory);
    VisualizeStaticResults(btrajectory);
    VisualizeStaticResults(ctrajectory);
    
    VisualizeStaticResults(btrajectory2);
    VisualizeStaticResults(ctrajectory2);

    if id>2000
        break;
    end
end

%% visualization for static results
function VisualizeStaticResults(trajectory)
global obstacles_ Nobs planning_scale_
nstep = length(trajectory.x);
figure;
%% plot obstacle
if Nobs>0
    for j=1:Nobs 
        vertex_x = obstacles_{j}.x;
        vertex_y = obstacles_{j}.y;
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

% transform the path to trajectory at maximum speed
function trajectory = PlanSpeedForStaticScenarios(x, y, theta, path_length)
    Nfe = length(x);
    % Judge velocity direction
    vdr = zeros(1, Nfe);
    for ii = 2 : (Nfe - 1)
        addtion = (x(ii+1) - x(ii)) * cos(theta(ii)) + (y(ii+1) - y(ii)) * sin(theta(ii));
        if (addtion > 0)
            vdr(ii) = 1;
        else
            vdr(ii) = -1;
        end
    end
    v = zeros(1, Nfe);
    a = zeros(1, Nfe);
    dt = path_length / Nfe;
    for ii = 2 : Nfe
        v(ii) = vdr(ii) * sqrt(((x(ii) - x(ii-1)) / dt)^2 + ((y(ii) - y(ii-1)) / dt)^2);
    end
    for ii = 2 : Nfe
        a(ii) = (v(ii) - v(ii-1)) / dt;
    end
    phi = zeros(1, Nfe);
    omega = zeros(1, Nfe);
    global vehicle_kinematics_ vehicle_geometrics_
    phi_max = vehicle_kinematics_.vehicle_phi_max;
    omega_max = vehicle_kinematics_.vehicle_omega_max;
    for ii = 2 : (Nfe-1)
        phi(ii) = atan((theta(ii+1) - theta(ii)) * vehicle_geometrics_.vehicle_wheelbase / (dt * v(ii)));
        if (phi(ii) > phi_max)
            phi(ii) = phi_max;
        elseif (phi(ii) < -phi_max)
            phi(ii) = -phi_max;
        end
    end
    for ii = 2 : (Nfe-1)
        omega(ii) = (phi(ii+1) - phi(ii)) / dt;
        if (omega(ii) > omega_max)
            omega(ii) = omega_max;
        elseif (omega(ii) < -omega_max)
            omega(ii) = -omega_max;
        end
    end
    trajectory.x=x; trajectory.y=y; trajectory.theta=theta; trajectory.v=v; trajectory.a=a; trajectory.phi=phi; trajectory.omega=omega;
end