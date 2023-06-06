%  This is the template for comparing different planners in dynamic structured environments. 
%  Copyright (C) 2020 Yuqing Guo. All rights reserved.
%  2022.04.02
% ==============================================================================
% ==============================================================================
clc;clear; close all; clear global var;
addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Graphbased');
addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Graphbased\SimpleStateLattice');
addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Graphbased\ImprovedStateLattice');
addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Samplingbased\InformedRRTStar');
addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Curvesbased');
addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\Planners\Optimizationbased');
addpath('D:\code library for trajectory planning\AutonomousDrivingTrajectoryPlanning\Matlab\ConvertPathToTrajectory');

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
planning_scale_.xmin=0; planning_scale_.xmax=40;planning_scale_.ymin=0;planning_scale_.ymax=6*vehicle_geometrics_.vehicle_width; %space is a rectange, [lx,ux],[ly,uy]
planning_scale_.x_scale = planning_scale_.xmax - planning_scale_.xmin;
planning_scale_.y_scale = planning_scale_.ymax - planning_scale_.ymin;

%% vehicle initial and terminal states settings
global vehicle_TPBV_
vehicle_TPBV_.x0=4; vehicle_TPBV_.y0=0.5*planning_scale_.ymax;vehicle_TPBV_.theta0=0;vehicle_TPBV_.v0=10;vehicle_TPBV_.phi0=0;vehicle_TPBV_.a=0;vehicle_TPBV_.omega0=0;
vehicle_TPBV_.xtf=planning_scale_.xmax-8; vehicle_TPBV_.ytf=5/6*planning_scale_.ymax;vehicle_TPBV_.thetatf=0;vehicle_TPBV_.vtf=10;vehicle_TPBV_.phitf=0;vehicle_TPBV_.a=0;vehicle_TPBV_.omegatf=0;

%% setting for the generated discreted map/graph (e.g., hybrid astar)
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
%% genearate obstacles
global obstacles_ Nobs 
global margin_obs_ % margin_obs_ for dilated obstacles
margin_obs_=0.5;
Nobs = 0; % the number for static obstacles
obstacles_ = GenerateStaticObstacles();

%% PVD: path and velocity decomposition
% plan path: lattice
% Hybrid A* 
t_h0=tic;
% [x, y, theta, path_length, completnessflag] = PlanAStarPath();
[x, y, theta, path_length, completnessflag] = PlanHybridAStarPath();
t_h = toc(t_h0);
% % SimpleLattice
% t_si0=tic;
% [x, y, theta, path_length_si, completnessflag] = PlanSimpleLatticePath(1, "look_up_table_153244.txt", "insert_points.txt");
% t_si = toc(t_si0);
% ImprovedLattice
% t_i0=tic;
% [x, y, theta, path_length, completnessflag] = PlanImprovedLatticePath(1, "look_up_table_121628.txt", "insert_points.txt");
% t_i = toc(t_i0);

%% st_graph settings
global st_graph_search_
st_graph_search_.k = 2;
st_graph_search_.num_nodes_s = num_nodes_s;
st_graph_search_.num_nodes_t = st_graph_search_.k*num_nodes_s;
st_graph_search_.multiplier_H_for_A_star = 2.0;
st_graph_search_.penalty_for_inf_velocity = 4;
st_graph_search_.max_t = round(path_length * 2);
st_graph_search_.max_s = path_length;
st_graph_search_.resolution_s = path_length / st_graph_search_.num_nodes_s;
st_graph_search_.resolution_t = st_graph_search_.max_t / st_graph_search_.num_nodes_t;
global dynamic_obs
dynamic_obs = GenerateDynamicObstacles();


% plan veolcity: the first time is the time for A* and the second time is the time for OP
trajectory = PlanSpeedForDynamicScenario(x, y, theta);
VisualizeDynamicResultsForDynamicScenarios(trajectory);

%% LLD: lateral and longitudinal decomposition
% plan lateral: bezier
% plan longitudnal: bezier

%% Bezier
t_b0=tic;
[x, y, theta, path_length_b, completnessflag] = PlanBezierPath();
trajectory_b = PlanSpeedForStaticScenario(x, y, theta, path_length_b);
t_b = toc(t_b0);

%% No decomposition: A* + SLIFS
t_a0=tic;
[x, y, theta, path_length_a, completnessflag] = PlanAStarPath();
trajectory_a = PlanSpeedForStaticScenario(x, y, theta, path_length_a);
t_a = toc(t_a0);
t_s0=tic;
collision_choice = 2; %choose CFS
[trajectory_s, isFeasible] = PlanL1SCPTrajectory(trajectory_a,collision_choice);
path_length_s = 0;
for i = 2:length(trajectory_s.theta)
    path_length_s = path_length_s + hypot(y(i) - y(i - 1), x(i) - x(i - 1));
end
t_s = toc(t_s0)+t_a; % it includes choosing the initial guess

%% show comparison results
path_runtime=['Runtime: Hybrid A*: ',num2str(t_h),' CLRRT*: ',num2str(t_cl),' Simple Lattice: ',num2str(t_si),' Improved Lattice: ',num2str(t_i),' Bezier: ',num2str(t_b),' SLIFS: ',num2str(t_s)];
disp(path_runtime);
path_length=['Length: Hybrid A*: ',num2str(path_length_h),' CLRRT*: ',num2str(path_length_cl),' Simple Lattice: ',num2str(path_length_si),' Improved Lattice: ',num2str(path_length_i),' Bezier: ',num2str(path_length_b),' SLIFS: ',num2str(path_length_s)];
disp(path_length);

%% plot trajectory comparison
figure;
for j=1:Nobs 
    vertex_x = obstacles_{j}.x;
    vertex_y = obstacles_{j}.y;
    fill(vertex_x,vertex_y,[0.7451 0.7451 0.7451]);
    hold on;
end
hh=plot(trajectory_h.x,trajectory_h.y,'LineWidth',1.5); % 'Color',[1 201/255 14/255]
hold on;
hcl=plot(trajectory_cl.x,trajectory_cl.y,'LineWidth',1.5); %[0 162/255 232/255]
hold on;
hsi=plot(trajectory_si.x,trajectory_si.y,'LineWidth',1.5); %'--' [181/255 230/255 29/255]
hold on;
hi=plot(trajectory_i.x,trajectory_i.y,'LineWidth',1.5); %[34/255 177/255 76/255]
hold on;
hb=plot(trajectory_b.x,trajectory_b.y,'LineWidth',1.5);
hold on;
hs=plot(trajectory_s.x,trajectory_s.y,'LineWidth',1.5);
hold on;
plot(vehicle_TPBV_.x0,vehicle_TPBV_.y0,'o','Color',[1 201/255 14/255],'LineWidth',1);
hold on;
plot(vehicle_TPBV_.xtf,vehicle_TPBV_.ytf,'p','Color',[1 201/255 14/255],'LineWidth',1);
hold on;
legend([hh hcl hsi hi hb hs],{'Hybrid A*','CLRRT*','Simple Lattice','Improved Lattice','Bezier','SLIFS',});
axis([planning_scale_.xmin,planning_scale_.xmax,planning_scale_.ymin,planning_scale_.ymax]);
xlabel('x (m)','FontSize',12);
ylabel('y (m)','FontSize',12);

%% generate random obstacles: obstacles do not overlap with the vehicle's initial/terminal state
function obstacle_cell = GenerateStaticObstacles()
global Nobs planning_scale_  vehicle_TPBV_  vehicle_geometrics_ margin_obs_ 
V_initial = CreateVehiclePolygon(vehicle_TPBV_.x0,vehicle_TPBV_.y0,vehicle_TPBV_.theta0);
V_terminal = CreateVehiclePolygon(vehicle_TPBV_.xtf,vehicle_TPBV_.ytf,vehicle_TPBV_.thetatf);
count = 0;
obj=[];
obstacle_cell = cell(1, Nobs);
lx=planning_scale_.xmin; ux=planning_scale_.xmax; ly=planning_scale_.ymin; uy=planning_scale_.ymax;
W=vehicle_geometrics_.vehicle_width;
L=vehicle_geometrics_.vehicle_length;

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
%     current_obstacle.x = [x,xru,xrd,xld,x];
%     current_obstacle.y = [y,yru,yrd,yld,y];
    current_obstacle.x = [x,xru,xrd,xld];
    current_obstacle.y = [y,yru,yrd,yld];
    obstacle_cell{1,count} = current_obstacle;
%     current_obstacle.A = CalculatePolygonArea(current_obstacle);
    obj = [obj;temp_obj];   
end
end

function obstacle_cell = GenerateDynamicObstacles()
global st_graph_search_ vehicle_geometrics_ 
halflen = 0.5*vehicle_geometrics_.vehicle_length;
halfwidth = 0.5*vehicle_geometrics_.vehicle_width;
lanewidth = 2*vehicle_geometrics_.vehicle_width;
halflanewidth = vehicle_geometrics_.vehicle_width;
len_b = [-halflen +halflen +halflen -halflen];
wid_b = [-halfwidth -halfwidth +halfwidth +halfwidth];
Nobs_dyn = 2;
dynamic_obs = cell(2,2); 
% structure, the first row stores the starting position of the dynamic obstacle, the second row stores the end position of the dynamic obstacle
% the first obstacle
dynamic_obs{1,1}.x=10+len_b; dynamic_obs{1,1}.y=halflanewidth+wid_b; % the center of the first lane
dynamic_obs{2,1}.x=30+len_b; dynamic_obs{2,1}.y=dynamic_obs{1,1}.y;
% the second obstacle
dynamic_obs{1,2}.x=10+len_b; dynamic_obs{1,2}.y=dynamic_obs{1,1}.y+lanewidth;
dynamic_obs{2,2}.x=30+len_b; dynamic_obs{2,2}.y=dynamic_obs{1,1}.y+lanewidth;
% % the third obstacle
% dynamic_obs{1,3}.x=10+len_b; dynamic_obs{1,3}.y=dynamic_obs{1,1}.y+2*lanewidth;
% dynamic_obs{2,3}.x=30+len_b; dynamic_obs{2,3}.y=dynamic_obs{1,1}.y+2*lanewidth;
% % the forth obstacle
% dynamic_obs{1,4}.x=10+len_b; dynamic_obs{1,4}.y=dynamic_obs{1,1}.y;
% dynamic_obs{2,4}.x=30+len_b; dynamic_obs{2,4}.y=dynamic_obs{1,1}.y+lanewidth;
% % the fifth obstacle
% dynamic_obs{1,5}.x=10+len_b; dynamic_obs{1,5}.y=dynamic_obs{1,1}.y+lanewidth;
% dynamic_obs{2,5}.x=30+len_b; dynamic_obs{2,5}.y=dynamic_obs{1,1}.y+2*lanewidth;

obstacle_cell = cell(st_graph_search_.num_nodes_t, Nobs_dyn);
for ii = 1 : Nobs_dyn
    dx = dynamic_obs{end,ii}.x(1) - dynamic_obs{1,ii}.x(1);
    dy = dynamic_obs{end,ii}.y(1) - dynamic_obs{1,ii}.y(1);
    for jj = 1 : st_graph_search_.num_nodes_t
        temp.x = dynamic_obs{1,ii}.x + dx / st_graph_search_.num_nodes_t * (jj - 1);
        temp.y = dynamic_obs{1,ii}.y + dy / st_graph_search_.num_nodes_t * (jj - 1);
        obstacle_cell{jj, ii} = temp;
    end
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

% transform the path to trajectory at maximum speed
function trajectory = PlanSpeedForStaticScenario(x, y, theta, path_length)
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

function VisualizeDynamicResultsForDynamicScenarios(trajectory)
x = trajectory.x; y = trajectory.y; theta = trajectory.theta;
global vehicle_TPBV_
global obstacles_ Nobs dynamic_obs
global planning_scale_
colorpool = [ 237, 28, 36;0, 162, 232;34, 177, 76;255, 127, 39 ] ./ 255;
number_of_frame = length( x );
vidObj = VideoWriter( 'TrajectoryForDynamicScenarios' );
vidObj.Quality = 100;
vidObj.FrameRate = 24;
open( vidObj );
figure;
hold on;axis equal;box on;grid on;
axis( [ planning_scale_.xmin,planning_scale_.xmax, planning_scale_.ymin, planning_scale_.ymax ] );
set( gcf, 'outerposition', get( 0, 'screensize' ) );
x0 = vehicle_TPBV_.x0;
y0 = vehicle_TPBV_.y0;
theta0 = vehicle_TPBV_.theta0;
xtf = vehicle_TPBV_.xtf;
ytf = vehicle_TPBV_.ytf;
thetatf = vehicle_TPBV_.thetatf;

for ii = 1:number_of_frame
    Arrow( [ x0, y0 ], [ x0 + cos( theta0 ), y0 + sin( theta0 ) ], 'Length', 16, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 2 );
    Arrow( [ xtf, ytf ], [ xtf + cos( thetatf ), ytf + sin( thetatf ) ], 'Length', 16, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 2 );
    %% plot static obstacles
    if Nobs>0
        for j=1:Nobs 
            vertex_x = obstacles_{j}.x;
            vertex_y = obstacles_{j}.y;
            fill(vertex_x,vertex_y,[0.7451 0.7451 0.7451]);
        end
    end
    %% plot dynamic obstacles
    for jj = 1:size( dynamic_obs, 2 )
        obs = dynamic_obs{ ii, jj };
        fill( obs.x, obs.y, [ 125, 125, 125 ] ./ 255 );
    end
    h0 = get( gca, 'children' );
    xc = x( ii );
    yc = y( ii );
    tc = theta( ii );
    V = CreateVehiclePolygon( xc, yc, tc );
    plot(V.x, V.y,'Color', colorpool( 2, : ),'LineWidth',1); 
    h1 = get( gca, 'children' );
    plot( x( 1:ii ), y( 1:ii ), 'b' );
    h2 = get( gca, 'children' );
    M( ii ) = getframe;
    writeVideo( vidObj, M( ii ) );
    if ( ii ~= number_of_frame )
        delete( h1 );
        delete( h2 );
    end
end
close( vidObj );
end
