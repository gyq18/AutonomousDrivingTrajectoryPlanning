%  This code generates dataset for learning-based planners, i.e.,
%  supervised learning and reinforcement learning.
%  Copyright (C) 2020 Yuqing Guo. All rights reserved.
%  2022.06.02
% ==============================================================================
% ==============================================================================
% Optimization-based trajectory plannes include these parts:
% 1. a good initial guess:addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Graphbased');
% 2. convex safe corridors;
% 3. solver--fmincon,cplexqp;
% ==============================================================================
% ==============================================================================
clc;clear; close all;
addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Graphbased');
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
% vehicle_TPBV_.x0=4; vehicle_TPBV_.y0=4;vehicle_TPBV_.theta0=0;
vehicle_TPBV_.v0=0;vehicle_TPBV_.phi0=0;vehicle_TPBV_.a0=0;vehicle_TPBV_.omega0=0;
% vehicle_TPBV_.xtf=52; vehicle_TPBV_.ytf=52;vehicle_TPBV_.thetatf=0;
vehicle_TPBV_.vtf=0;vehicle_TPBV_.phitf=0;vehicle_TPBV_.atf=0;vehicle_TPBV_.omegatf=0;
% vehicle_TPBV_.x0=4; vehicle_TPBV_.y0=4;vehicle_TPBV_.theta0=0;vehicle_TPBV_.v0=0;vehicle_TPBV_.phi0=0;vehicle_TPBV_.a0=0;vehicle_TPBV_.omega0=0;
% vehicle_TPBV_.xtf=52; vehicle_TPBV_.ytf=52;vehicle_TPBV_.thetatf=0;vehicle_TPBV_.vtf=0;vehicle_TPBV_.phitf=0;vehicle_TPBV_.a0=0;vehicle_TPBV_.omegatf=0;

%% genearate random static obstacle
global obstacles_ Nobs 
global margin_obs_ % margin_obs_ for dilated obstacles
margin_obs_=2;
% Nobs = 10;
% obstacles_ = GenerateStaticObstacles_unstructured();
global costmap_

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

%% generate datasets: 2000
vehicle_TPBV_.x0=4; vehicle_TPBV_.y0=4;vehicle_TPBV_.theta0=0;

id=2000;
theta_r=[0,1/4,1/3,1/2,3/4,2/3,1];
for i=1:2000
    theta_tf = theta_r(rem(i,7)+1);
    vehicle_TPBV_.xtf=randi([4,54],1,1); vehicle_TPBV_.ytf=randi([4,54],1,1);vehicle_TPBV_.thetatf=pi*theta_tf;
%     Nobs = 14;
    Nobs = randi([2,20],1,1);
    obstacles_ = GenerateStaticObstacles_unstructured();
    
    [x, y, theta, path_length, completeness_flag] = PlanAStarPath();
    if completeness_flag
        trajectory = TransformPathToTrajectory(x, y, theta, path_length, transmethod_flag);
        [trajectory,isfeasible] = PlanL1SCPTrajectory(trajectory,collision_choice); %PlanL1SCPTrajectory(trajectory,collision_choice);
        obs_m = zeros(20,8);
        for k = 1:Nobs
            obs_m(k,1:4)=obstacles_{k}.x;
            obs_m(k,5:8)=obstacles_{k}.y;
        end
            
        task_m=[vehicle_TPBV_.x0,vehicle_TPBV_.xtf;vehicle_TPBV_.y0,vehicle_TPBV_.ytf;vehicle_TPBV_.theta0,vehicle_TPBV_.thetatf;
            vehicle_TPBV_.v0,vehicle_TPBV_.vtf;vehicle_TPBV_.phi0,vehicle_TPBV_.phitf;vehicle_TPBV_.a0,vehicle_TPBV_.atf;
            vehicle_TPBV_.omega0,vehicle_TPBV_.omegatf];

        %% if the generated trajectory satisfy kinematic constraints
        if isfeasible
            iscollision=JudgeCollision(trajectory);
            if iscollision == 0
                traj_m=zeros(length(trajectory.x),6);  
                traj_m(:,1)=trajectory.x; traj_m(:,2)=trajectory.y;  traj_m(:,3)=trajectory.theta; traj_m(:,4)=trajectory.v;
                traj_m(:,5)=trajectory.phi; traj_m(:,6)=trajectory.tf;

                id=id+1;
%                 mapfile = [pwd, '\dataset\map\', num2str(id), '.csv'];
                obsfile = [pwd, '\dataset\obs\', num2str(id), '.csv'];
                taskfile = [pwd, '\dataset\task\', num2str(id), '.csv'];
                trajfile = [pwd, '\dataset\traj\', num2str(id), '.csv'];
%                 writematrix(costmap_,mapfile);
                writematrix(obs_m,obsfile);
                writematrix(task_m,taskfile);
                writematrix(traj_m,trajfile);
            end
        end
    end
    if id>2000
        break;
    end
end

%% visualize planning results
% VisualizeStaticResults(trajectory);
% VisualizeDynamicResults(trajectory);


%% rejudge whether the generated trajectory collide with obstacles
function iscollision=JudgeCollision(trajectory)
global obstacles_ Nobs
iscollision=0;
nstep=length(trajectory.x);

obj=[];
for j=1:Nobs 
    vertex_x = obstacles_{j}.x;
    vertex_y = obstacles_{j}.y;
    obj=[vertex_x;vertex_y];
end

    for i=1:nstep
        %% vehicle body
        px = trajectory.x(i);
        py = trajectory.y(i);
        pth = trajectory.theta(i);
        V = CreateVehiclePolygon(px,py,pth);
        temp_v = [V.x;V.y];
        flag1 = checkObj_linev(temp_v(:,1),temp_v(:,2),obj);
        flag2 = checkObj_linev(temp_v(:,4),temp_v(:,3),obj);
        flag3 = checkObj_linev(temp_v(:,1),temp_v(:,3),obj);
        flag4 = checkObj_linev(temp_v(:,2),temp_v(:,4),obj);
        if flag1+flag2+flag3+flag4 > 0
            iscollision = 1;
        end
        
    end
end


% %% visualization for static results
% function VisualizeStaticResults(trajectory)
% global obstacles_ Nobs planning_scale_
% nstep = length(trajectory.x);
% figure;
% %% plot obstacle
% if Nobs>0
%     for j=1:Nobs 
%         vertex_x = obstacles_{j}.x;
%         vertex_y = obstacles_{j}.y;
%         fill(vertex_x,vertex_y,[0.7451 0.7451 0.7451]);
%         hold on;
%     end
% end
% %% plot the planned trajectory
% plot(trajectory.x,trajectory.y,'.-','Color',[1 127/255 39/255],'LineWidth',1); 
% hold on;
% %% plot vehicle body
% for i= 1:nstep
%     px = trajectory.x(i);
%     py = trajectory.y(i);
%     pth = trajectory.theta(i);
%     V = CreateVehiclePolygon(px,py,pth);
%     plot(V.x, V.y,'Color',[153/255 217/255 234/255],'LineWidth',1); 
%     hold on;
% end
% %% plot start and terminal point
% plot(trajectory.x(1),trajectory.y(1),'o','Color',[1 201/255 14/255],'LineWidth',1);
% hold on;
% plot(trajectory.x(nstep),trajectory.y(nstep),'p','Color',[1 201/255 14/255],'LineWidth',1);
% hold on;
% axis([planning_scale_.xmin,planning_scale_.xmax,planning_scale_.ymin,planning_scale_.ymax]);
% axis equal;
% xlabel('x (m)','FontSize',12);
% ylabel('y (m)','FontSize',12);
% hold off;
% end

%% visualization for static results
function VisualizeDynamicResults(trajectory)
warning off
hold on;
global obstacles_ Nobs planning_scale_
nstep = length(trajectory.x);

axis equal
box on
set( gcf, 'outerposition', get( 0, 'screensize' ) );

for i= 1:nstep
    %% plot obstacle
    if Nobs>0
        for j=1:Nobs 
            vertex_x = obstacles_{j}.x;
            vertex_y = obstacles_{j}.y;
            fill(vertex_x,vertex_y,[0.7451 0.7451 0.7451]);
        end
    end
    
    axis equal; axis([planning_scale_.xmin,planning_scale_.xmax,planning_scale_.ymin,planning_scale_.ymax]);
    h1 = get( gca, 'children' );
    
    %% plot the planned trajectory
    plot(trajectory.x,trajectory.y,'.-','Color',[1 127/255 39/255],'LineWidth',1); 

    %% plot vehicle body
    px = trajectory.x(i);
    py = trajectory.y(i);
    pth = trajectory.theta(i);
    V = CreateVehiclePolygon(px,py,pth);
    plot(V.x, V.y,'Color',[153/255 217/255 234/255],'LineWidth',1); 

    h2 = get( gca, 'children' );
    M( i ) = getframe;
    if ( i ~= nstep )
        delete( h1 );
        delete( h2 );
    end
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

%% generate random obstacles: obstacles do not overlap with the vehicle's initial/terminal state
function obstacle_cell = GenerateStaticObstacles_unstructured()
global Nobs planning_scale_  vehicle_TPBV_  vehicle_geometrics_ margin_obs_ 
V_initial = CreateVehiclePolygon(vehicle_TPBV_.x0,vehicle_TPBV_.y0,vehicle_TPBV_.theta0);
V_terminal = CreateVehiclePolygon(vehicle_TPBV_.xtf,vehicle_TPBV_.ytf,vehicle_TPBV_.thetatf);
count = 0;
obj=[];
obstacle_cell = cell(1, Nobs);
lx=planning_scale_.xmin; ux=planning_scale_.xmax; ly=planning_scale_.ymin; uy=planning_scale_.ymax;
W=vehicle_geometrics_.vehicle_width+2;
L=vehicle_geometrics_.vehicle_length+2;

while count<Nobs
    % obstacle point
    x = (ux-lx)*rand+lx;
    y = (uy-ly)*rand+ly;
    theta = 2*pi*rand-pi;
    xru = x+L*rand*cos(theta);
    yru = y+L*rand*sin(theta);
    xrd = xru+W*rand*sin(theta);
    yrd = yru-W*rand*cos(theta);
    xld = x+W*rand*sin(theta);
    yld = y-W*rand*cos(theta);
    if xru<lx || xru>ux || xrd<lx || xrd>ux || xld<lx || xld>ux
        continue;
    elseif yru<ly || yru>uy || yrd<ly || yrd>uy || yld<ly|| yld>uy
        continue;
    end
    temp_obj = [x,xru,xrd,xld;y,yru,yrd,yld];
        
    % check the initial/terminal point in the obstacle 
    xv=[x-margin_obs_,xru+margin_obs_,xrd+margin_obs_,xld-margin_obs_,x-margin_obs_]; yv=[y+margin_obs_,yru+margin_obs_,yrd-margin_obs_,yld-margin_obs_,y+margin_obs_];
    temp_obj_margin = [x-margin_obs_,xru+margin_obs_,xrd+margin_obs_,xld-margin_obs_,x-margin_obs_;y+margin_obs_,yru+margin_obs_,yrd-margin_obs_,yld-margin_obs_,y+margin_obs_];
   
    if inpolygon(vehicle_TPBV_.x0,vehicle_TPBV_.y0,xv,yv) > 0 || inpolygon(vehicle_TPBV_.xtf,vehicle_TPBV_.ytf,xv,yv) > 0 
        continue;
    end
   
    label_obj = 0;
    
    % if the generated obstacle overlaps with other obstacles
    if count > 0
        flag1 = checkObj_linev(temp_obj(:,1),temp_obj(:,2),obj);
        flag2 = checkObj_linev(temp_obj(:,4),temp_obj(:,3),obj);
        flag3 = checkObj_linev(temp_obj(:,1),temp_obj(:,3),obj);
        flag4 = checkObj_linev(temp_obj(:,2),temp_obj(:,4),obj);
        if flag1+flag2+flag3+flag4 > 0
            label_obj = 1;
        end
    end
    
    % if the generated obstacle overlaps with initial and terminal states
    vehicle_state=[V_initial.x(1:4);V_initial.y(1:4);V_terminal.x(1:4);V_terminal.y(1:4)];
    if count > 0
        flag1 = checkObj_linev(temp_obj_margin(:,1),temp_obj_margin(:,2),vehicle_state);
        flag2 = checkObj_linev(temp_obj_margin(:,4),temp_obj_margin(:,3),vehicle_state);
        flag3 = checkObj_linev(temp_obj_margin(:,1),temp_obj_margin(:,3),vehicle_state);
        flag4 = checkObj_linev(temp_obj_margin(:,2),temp_obj_margin(:,4),vehicle_state);
        if flag1+flag2+flag3+flag4 > 0
            label_obj = 1;
        end
    end
    
    % if the generated obstacle overlaps with other obstacles
    if label_obj > 0
        continue;
    end
    
    % if the generated obstacle is effective
    count = count + 1;
    current_obstacle.x = [x,xru,xrd,xld];
    current_obstacle.y = [y,yru,yrd,yld];
    obstacle_cell{1,count} = current_obstacle;
    obj = [obj;temp_obj];   
end
end

function result = checkObj_linev(x1,x2,obj)
    %% Determine whether the line segment formed by x1 and x2 intersects the obstacle
    %% The obstacle here is mainly a polygon with four vertices
    result = 1;
    % Number of obstacles
    [n,~] = size(obj);
    nobj = n/2;
    for i = 1:nobj
        index = (i-1) * 2 + 1:i * 2;
        temp_new_obj = obj(index,:);
        %% First determine whether the vertex is inside the obstacle
        result1 = checkObj_point(x1,temp_new_obj);
        result2 = checkObj_point(x2,temp_new_obj);
        if result1 == 0 && result2 == 0 % If none of the line segment endpoints are inside the obstacle
            result = 0;
        else
            result = 1;
            break;
        end
        %% If the vertices are both outside the obstacle, determine whether the two diagonals intersect the line segment
        % Direction of the line segment
        v1 = x2 - x1;
        %% Diagonal1
        c1 = temp_new_obj(:,1);
        c2 = temp_new_obj(:,3);
        % Direction vector of the diagonal
        v2 = c2 - c1;
        % If two lines are parallel, skip
        norm_dist1 = norm(v1 - v2);
        norm_dist2 = norm(v1 + v2);
        if norm_dist1 < 1e-6 || norm_dist2 < 1e-6
            result = 0;
        else
            % Calculate the intersection of two lines
            t1 = (v2(2)*(c1(1) - x1(1)) + v2(1)*(x1(2) -c1(2))) / (v1(1)*v2(2) - v2(1)*v1(2));
            t2 = (v1(2)*(x1(1) - c1(1)) + v1(1)*(c1(2) - x1(2))) / (v2(1)*v1(2) - v1(1)*v2(2));
            if t1>=0 && t1<=1 && t2>=0 && t2<=1 % Diagonal intersects the line segment
                result = 1;
                break;
            end        
        end

         %% Diagonal2
        c1 = temp_new_obj(:,2);
        c2 = temp_new_obj(:,4);
       % Direction vector of the diagonal
        v2 = c2 - c1;
       % If two lines are parallel, skip
        norm_dist1 = norm(v1 - v2);
        norm_dist2 = norm(v1 + v2);
        if norm_dist1 < 1e-6 || norm_dist2 < 1e-6
            result = 0;
        else
           % Calculate the intersection of two lines
            t1 = (v2(2)*(c1(1) - x1(1)) + v2(1)*(x1(2) -c1(2))) / (v1(1)*v2(2) - v2(1)*v1(2));
            t2 = (v1(2)*(x1(1) - c1(1)) + v1(1)*(c1(2) - x1(2))) / (v2(1)*v1(2) - v1(1)*v2(2));
            if t1>=0 && t1<=1 && t2>=0 && t2<=1 % Diagonal intersects the line segment
                result = 1;
                break;
            end        
        end
    end
end

function result = checkObj_point(xr,obj)
    %% Determine if xr is inside the obstacle
    % The obstacle here is mainly a polygon with four vertices
    result = 0;
    ncorner = 4;
    % Calculate the area of the obstacle area_obj and the sum of the four triangles areaarea, if they are equal, it means xr is inside the obstacle
    area = 0; area_obj = 0;
    % triArea is used to calculate the area of a triangle
    for i=1:ncorner
        area = area + triArea(xr,obj(:,i),obj(:,mod(i,ncorner)+1));
    end
    for i=2:ncorner-1
        area_obj = area_obj + triArea(obj(:,1),obj(:,i),obj(:,mod(i,ncorner)+1));
    end
    % if the reference point is inside the obstacle, then area = polyarea
    if norm(area_obj-area) < 0.01
        result = 1;
    end
end

function area = triArea(p1,p2,p3)
a = norm(p1-p2);
b = norm(p1-p3);
c = norm(p2-p3);
half = (a+b+c)/2;
area = sqrt(half*(half-a)*(half-b)*(half-c));
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