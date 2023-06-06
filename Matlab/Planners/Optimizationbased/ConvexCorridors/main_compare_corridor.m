%% the main code is for comparing different convex set methods (path planning)
clc; clear; close all; clear global var;
     
% randomized repetition of the experiment
num_experiment = 4;
%% statistics (respectively, feasible (1,0), number of outer iterations, total time, time to find convex feasible domain, time to solve optimization problem in convex feasible domain, objective function)
all_CFS = zeros(num_experiment,6); 
all_box = zeros(num_experiment,6); 
all_bubble = zeros(num_experiment,6); 

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
vehicle_TPBV_.x0=4; vehicle_TPBV_.y0=4;vehicle_TPBV_.theta0=0;vehicle_TPBV_.v0=0;vehicle_TPBV_.phi0=0;vehicle_TPBV_.a=0;vehicle_TPBV_.omega0=0;
vehicle_TPBV_.xtf=54; vehicle_TPBV_.ytf=44;vehicle_TPBV_.thetatf=0;vehicle_TPBV_.vtf=0;vehicle_TPBV_.phitf=0;vehicle_TPBV_.a=0;vehicle_TPBV_.omegatf=0;

%% genearate random static obstacle
global obstacles_ Nobs 
global margin_obs_ % margin_obs_ for dilated obstacles
margin_obs_=0.5;

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

%% set the cost function for optimization problem
% The distance metric between the original path and the new path
dim = 2;
Q1 = eye(num_nodes_s*dim);
% The velocity
Vdiff = eye(num_nodes_s*dim)-diag(ones(1,(num_nodes_s-1)*dim),dim);
Q2 = Vdiff(1:(num_nodes_s-1)*dim,:)'*Q1(1+dim:end,1+dim:end)*Vdiff(1:(num_nodes_s-1)*dim,:);
% The accelaration
Adiff = Vdiff-diag(ones(1,(num_nodes_s-1)*dim),dim)+diag(ones(1,(num_nodes_s-2)*dim),dim*2);
Q3 = Adiff(1:(num_nodes_s-2)*dim,:)'*Adiff(1:(num_nodes_s-2)*dim,:);
% The weight
cref = [1,1,1];
cabs = [1,1,1];    
% The total costj
Qref = Q1*cref(1)/num_nodes_s+Q2*cref(2)*(num_nodes_s-1)+Q3*cref(3)*(num_nodes_s-1)^4/(num_nodes_s-2);
Qabs = Q1*cabs(1)/num_nodes_s+Q2*cabs(2)*(num_nodes_s-1)+Q3*cabs(3)*(num_nodes_s-1)^4/(num_nodes_s-2);
Qe = Qref+Qabs;
%The boundary constraint
Aeq = zeros(2*dim,num_nodes_s*dim);
Aeq(0*dim+1:1*dim,1:dim) = eye(dim);
Aeq(1*dim+1:2*dim,(num_nodes_s-1)*dim+1:num_nodes_s*dim) = eye(dim);
beq = [vehicle_TPBV_.x0;vehicle_TPBV_.y0;vehicle_TPBV_.xtf;vehicle_TPBV_.ytf];

%Virtual shortest initial trajectory for penalty - direct line from start to finish
path = [];
start = [vehicle_TPBV_.x0;vehicle_TPBV_.y0];
ending = [vehicle_TPBV_.xtf;vehicle_TPBV_.ytf];
for i = 1:num_nodes_s
    path(:,i) = (num_nodes_s-i)/(num_nodes_s-1)*start+(i-1)/(num_nodes_s-1)*ending;
end
oripath = [];
for i=1:num_nodes_s
    oripath = [oripath;path(:,i)];
end

% for cplexqp
% options = cplexoptimset;
% options.Display = 'off';
options = optimset('display','off','Algorithm','interior-point');

max_iter = 10;

%% compare experiment
for kkk = 1:num_experiment %   
    Nobs = 2;
    obstacles_ = GenerateStaticObstacles();
    obj = [];
    for i = 1:Nobs
        obj = [obj; obstacles_{i}.x; obstacles_{i}.y];
    end
    num_collision_max = num_nodes_s * Nobs;
    [x, y, ~, ~, completeness_flag] = PlanAStarPath();
    if completeness_flag 
        path_initial = zeros(num_nodes_s*dim,1);
        path_initial(1:2:end) = x;
        path_initial(2:2:end) = y;
       
        %% CFS
        timecfs=0;
        timeop=0;
        alltimecfs0=tic;
        path_k = path_initial;
        for k = 1:max_iter 
             timecfs0=tic;
             A = zeros(num_collision_max, dim*num_nodes_s);
             b = zeros(num_collision_max, 1);
             counter_collision = 1;
             for i = 1:num_nodes_s
                indexi = (i - 1) * dim + 1:i * dim;
                xnr = path_k(indexi); % xnr is a column vector
                [tempA, tempb] = FindCFS(xnr, obj);
                num_tempA = length(tempb);      
                A(counter_collision:counter_collision - 1 + num_tempA, 2 * (i - 1) + 1:2 * i) = tempA;
                b(counter_collision:counter_collision - 1 + num_tempA) = tempb-margin_obs_;      
                counter_collision = counter_collision + num_tempA;
             end
            timecfs=timecfs+toc(timecfs0); 
            timeop0=tic;
%             [pathnew,fval,~,~,~] = ...
%             cplexqp(Qe,(-Qref*oripath)',A,b,Aeq(:,1:size(Qe,1)),beq,[],[],path_k,options);
            [pathnew,fval,~,output] = fmincon(@(x) fun(x,Qe,Qref,oripath),path_k,A,b,Aeq(:,1:size(Qe,1)),beq,[],[],[],options);   
            timeop=timeop+toc(timeop0);
            if isempty(pathnew)
                continue;  
            end
            diff = norm(path_k-pathnew);
            if diff < 0.001*num_nodes_s*(dim)
                break
            end
            path_k = pathnew;
        end
        alltimecfs=toc(alltimecfs0); %Total time
        if isempty(pathnew)==0 
            all_CFS(kkk,1)=1; % Feasible
            all_CFS(kkk,2)=k; % Iterations of the outer loop
            all_CFS(kkk,3)=alltimecfs; % Total time
            all_CFS(kkk,4)=timecfs/k; % Time to find a convex feasible set
            all_CFS(kkk,5)=timeop/k; % The time to solve the optimization problem in the convex feasible domain
            all_CFS(kkk,6)=fval; % Objective function       
        end
        path_cfs = pathnew;
            
        %% Box
        timebox=0;
        timeop=0;
        alltimebox0=tic;
        path_k = path_initial;
        for k = 1:max_iter 
             timebox0=tic;
             A = zeros(num_collision_max, dim*num_nodes_s);
             b = zeros(num_collision_max, 1);
             counter_collision = 1;
             for i = 1:num_nodes_s
                indexi = (i - 1) * dim + 1:i * dim;
                xnr = path_k(indexi); % xnr is a column vector
                [tempA, tempb] = FindCFS(xnr, obj);
                num_tempA = length(tempb);      
                A(counter_collision:counter_collision - 1 + num_tempA, 2 * (i - 1) + 1:2 * i) = tempA;
                b(counter_collision:counter_collision - 1 + num_tempA) = tempb;      
                counter_collision = counter_collision + num_tempA;
             end
            timebox=timebox+toc(timebox0); 
            timeop0=tic;
%             [pathnew,fval,~,~,~] = ...
%             cplexqp(Qe,(-Qref*oripath)',A,b,Aeq(:,1:size(Qe,1)),beq,[],[],path_k,options);
            [pathnew,fval,~,output] = fmincon(@(x) fun(x,Qe,Qref,oripath),path_k,A,b,Aeq(:,1:size(Qe,1)),beq,[],[],[],options);   
            timeop=timeop+toc(timeop0);
            if isempty(pathnew)
                continue;  
            end
            diff = norm(path_k-pathnew);
            if diff < 0.001*num_nodes_s*(dim)
                break
            end
            path_k = pathnew;
        end
        alltimebox=toc(alltimebox0); %Total time
        if isempty(pathnew)==0 
            all_box(kkk,1)=1; % Feasible
            all_box(kkk,2)=k; % Iterations of the outer loop
            all_box(kkk,3)=alltimebox; % Total time
            all_box(kkk,4)=timebox/k; % Time to find a convex feasible set
            all_box(kkk,5)=timeop/k; % The time to solve the optimization problem in the convex feasible domain
            all_box(kkk,6)=fval; % Objective function       
        end 
        path_box = pathnew;
         
        %% Bubble (cannot )
        timebubble=0;
        timeop=0;
        alltimebubble0=tic;
        path_k = path_initial;
        for k = 1:max_iter 
             timebubble0=tic;
%              Q = cell(1,num_nodes_s); % https://stackoom.com/question/3OFOl
%              L = zeros(dim*num_nodes_s, num_nodes_s);
%              R = zeros(1,num_nodes_s);
%              for i = 1:num_nodes_s
%                 Q{i} = zeros(dim*num_nodes_s, dim*num_nodes_s);
%                 indexi = (i - 1) * dim + 1:i * dim;
%                 xnr = path_k(indexi); % xnr is a column vector
%                 [x,y,r] = FindBubble(xnr, obj);
%                 Q{i}((i-1)*dim+1:i*dim,(i-1)*dim+1:i*dim) = eye(dim);
%                 L((i-1)*dim+1,i) = -2*x; 
%                 L(i*dim,i) = -2*y;
%                 R(i)=r^2-x^2-y^2;
%              end
            mu1=zeros(num_nodes_s,1);mu2=zeros(num_nodes_s,1);r=zeros(num_nodes_s,1);
            for i=1:num_nodes_s
                xnr = path_k(indexi);
                [mu1(i),mu2(i),r(i)] = FindBubble(xnr, obj);
            end
            timebubble=timebubble+toc(timebubble0); 
            timeop0=tic;
%             [pathnew,fval,~,~,~] = ...
%             cplexqcp(Qe,(-Qref*oripath)',A,b,Aeq(:,1:size(Qe,1)),beq,L,Q,R,path_k,options); % L'*x + x'*Q*x <= R
            [pathnew,fval,~,output] = fmincon(@(x) fun(x,Qe,Qref,oripath),path_k,A,b,Aeq(:,1:size(Qe,1)),beq,[],[],@(x)bubblecon(x,num_nodes_s,mu1,mu2,r),options);   
            timeop=timeop+toc(timeop0);
            if isempty(pathnew)
                continue;  
            end
            diff = norm(path_k-pathnew);
            if diff < 0.001*num_nodes_s*(dim)
                break
            end
            path_k = pathnew;
        end
        alltimebubble=toc(alltimebubble0); %Total time
        if isempty(pathnew)==0 
            all_bubble(kkk,1)=1; % Feasible
            all_bubble(kkk,2)=k; % Iterations of the outer loop
            all_bubble(kkk,3)=alltimebubble; % Total time
            all_bubble(kkk,4)=timebubble/k; % Time to find a convex feasible set
            all_bubble(kkk,5)=timeop/k; % The time to solve the optimization problem in the convex feasible domain
            all_bubble(kkk,6)=fval; % Objective function       
        end 
        path_bubble = pathnew;
    end    
end

%% result
fea_cfs=sum(all_CFS(:,1));fea_box=sum(all_box(:,1));fea_bubble=sum(all_bubble(:,1));
out_cfs=mean(all_CFS(:,2));out_box=mean(all_box(:,2));out_bubble=mean(all_bubble(:,2)); % iterations
time_cfs=mean(all_CFS(:,3));time_box=mean(all_box(:,3));time_bubble=mean(all_bubble(:,3));% total time
find_cfs=mean(all_CFS(:,4));find_box=mean(all_box(:,4));find_bubble=mean(all_bubble(:,4));% time for finding cfs
solve_cfs=mean(all_CFS(:,5));solve_box=mean(all_box(:,5));solve_bubble=mean(all_bubble(:,5)); % time for solving op
fval_cfs=mean(all_CFS(:,6));fval_box=mean(all_box(:,6));fval_bubble=mean(all_bubble(:,6)); % objective function   
disp(['The feasibility times for cfs, box, bubble are: ',num2str(fea_cfs),' ,',num2str(fea_box),' ,',num2str(fea_bubble)]);
disp(['The iterationss for cfs, box, bubble are: ',num2str(out_cfs),' ,',num2str(out_box),' ,',num2str(out_bubble)]);
disp(['The all time for cfs, box, bubble are: ',num2str(time_cfs),' ,',num2str(time_box),' ,',num2str(time_bubble)]);
disp(['The time of find cfs for cfs, box, bubble are: ',num2str(find_cfs),' ,',num2str(find_box),' ,',num2str(find_bubble)]);
disp(['The time of optimiation for cfs, box, bubble are: ',num2str(solve_cfs),' ,',num2str(solve_box),' ,',num2str(solve_bubble)]);
disp(['The fval for cfs, box, bubble are: ',num2str(fval_cfs),' ,',num2str(fval_box),' ,',num2str(fval_bubble)]);

% %% plot results
% figure; 
% for j=1:Nobs
%     vertex_x = obj(2*j-1,:);
%     vertex_y = obj(2*j,:);
%     fill(vertex_x,vertex_y,'k');
%     hold on;
% end
% h1=plot(path_cfs(1:dim:end),path_cfs(2:dim:end));
% hold on;
% h2=plot(path_box(1:dim:end),path_box(2:dim:end));
% hold on;
% h3=plot(path_bubble(1:dim:end),path_cfs(2:dim:end));
% hold on;
% legend([h1,h2,h3],{'cfs','box','bubble'})
% 
% figure;
% subplot(2,2,1)
% boxplot([all_CFS(:,2) all_box(:,2) all_bubble(:,2)]);
% title('The iterations for cfs, box, bubble');
% 
% subplot(2,2,2)
% boxplot([all_CFS(:,3) all_box(:,3) all_bubble(:,3)]);
% title('The all time for cfs, box, bubble');
% 
% subplot(2,2,3)
% boxplot([all_CFS(:,4) all_box(:,4) all_bubble(:,4)]);
% title('The time of find cfs for cfs, box, bubble');
% 
% subplot(2,2,4)
% boxplot([all_CFS(:,5) all_box(:,5) all_bubble(:,5)]);
% title('The time of optimiation for cfs, box, bubble');
% 
% figure;
% boxplot([all_CFS(:,6) all_box(:,6)  all_bubble(:,6)]);
% title('The fval for cfs, box, bubble');

function f=fun(x,Qe,Qref,oripath)
f=0.5*x'*Qe*x-(Qref*oripath)'*x;
end

function [ce,ceq]=bubblecon(x,nstep,mu1,mu2,r)
     for i=1:nstep
         ce(i)=norm([x(2*i-1),x(2*i)]-[mu1(i),mu2(i)])-r(i);
%         ce(i)=(x(2*i-1)-mu1(i))^2+(x(2*i)-mu2(i))^2-r(i)^2;
     end
     ceq=[];
end

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

