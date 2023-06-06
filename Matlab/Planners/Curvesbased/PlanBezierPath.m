function [x, y, theta, path_length, completeness_flag,box_list] = PlanBezierPath()
global costmap_
costmap_ = CreateDilatedCostmap();
global hybrid_astar_ vehicle_TPBV_
end_config = [vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf];%4,4,0
start_config = [vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0];%32,32,0

k = 50;
source_vertex = start_config(1:2);
goal_vertex = end_config(1:2);
start_ind = Convert2DimConfigToIndex(source_vertex);
end_ind = Convert2DimConfigToIndex(goal_vertex);
if ~Is2DNodeValid(source_vertex,start_ind),error('source lies on an obstacle or outside map'); end
if ~Is2DNodeValid(goal_vertex,end_ind),error('goal lies on an obstacle or outside map'); end
if ~Is3DNodeValid(start_config),error('vehicle can not start path correctly');end
if ~Is3DNodeValid(end_config),error('vehicle can not end path correctly');end
vertex = [source_vertex;goal_vertex];

global planning_scale_
lx=planning_scale_.xmin; ux=planning_scale_.xmax; ly=planning_scale_.ymin; uy=planning_scale_.ymax;
while length(vertex) < k+2
    x = (ux-lx)*rand+lx;
    y = (uy-ly)*rand+ly;
    point_2dconfig = [x,y];
    point_2dindex = Convert2DimConfigToIndex(point_2dconfig);
    if Is2DNodeValid(point_2dconfig,point_2dindex)
        vertex = [vertex;point_2dconfig];
    end
end
edges = cell(k+2,1);
for i=1:k+2
    for j=i+1:k+2
        pt_s = vertex(i,:);
        pt_e = vertex(j,:);
        
        x0=vertex(i,1);
        y0=vertex(i,2);
        x1=vertex(j,1);
        y1=vertex(j,2);
        theta = atan2(y1-y0,x1-x0);
        V0 = CreateVehiclePolygon(x0,y0,theta);
        V1 = CreateVehiclePolygon(x1,y1,theta);
        x = [V0.x,V1.x];
        y = [V0.y,V1.y];
        [~, ind_xmin] = min(x);
        [~, ind_xmax] = max(x);
        [~, ind_ymin] = min(y);
        [~, ind_ymax] = max(y);
        fea_poly = [x(ind_xmin),y(ind_xmin);
                    x(ind_ymax),y(ind_ymax)
                    x(ind_xmax),y(ind_xmax);
                    x(ind_ymin),y(ind_ymin);
                    x(ind_xmin),y(ind_xmin)];
        Isfeasible_Line=collisioncheck(fea_poly);
        if Isfeasible_Line
            edges{i}=[edges{i};j];
            edges{j}=[edges{j};i];
        end
    end
end

openlist_ = [1 0 norm(vertex(1,:)-vertex(2,:)) 0+norm(vertex(1,:)-vertex(2,:)) -1];
disp(openlist_);
closed=[];

completeness_flag = 0;
path_length = 0;
iter = 0;

tic
while ((size(openlist_,1)>0) && (iter <= hybrid_astar_.max_iter) && (~completeness_flag) && (toc <= hybrid_astar_.max_time))
    iter = iter + 1;
    [~,Ind]=min(openlist_,[],1);
    cur_node=openlist_(Ind(4),:);% smallest cost element to process
    openlist_=[openlist_(1:Ind(4)-1,:);openlist_(Ind(4)+1:end,:)];
    if cur_node(1) == 2
        path_length=cur_node(4);
        completeness_flag = 1;break;
    end
    
    for mv=1:length(edges{cur_node(1),1}) %iterate through all edges from the node
        newVertex=edges{cur_node(1),1}(mv);
        
        if isempty(closed) || isempty(find(closed(:,1)==newVertex, 1))
            child_g = cur_node(2) + norm(vertex(cur_node(1),:)-vertex(newVertex,:));
            
            child_h = norm((vertex(2,:)-vertex(newVertex,:)));
            child_f = child_g+child_h;
            add=true;
            if length(find(openlist_(:,1)==newVertex))>=1
                I=find(openlist_(:,1)==newVertex);
                if openlist_(I,4)<child_f
                    add=false;
                else
                    openlist_(I,:)=[];
                    add=true;
                end
            end
            if add
                openlist_=[openlist_;newVertex child_g child_h child_f size(closed,1)+1];
            end
        end
    end
    closed=[closed;cur_node];
end
disp("time:");
toc;
path = [vertex(cur_node(1),:)];
prev = cur_node(5);
while prev > 0
    path = [vertex(closed(prev,1),:);path];
    prev=closed(prev,5);
end
%% path resample
path = Resample(path);

%% get corrider and time
[box_list, corridor,ts] = getcorridor(path);
n_seg = size(corridor,1);
n_order=6;
%% trajectory plan
poly_coef_x = MinimumSnapCorridorBezierSolver(1, corridor, ts, n_order);
poly_coef_y = MinimumSnapCorridorBezierSolver(2, corridor, ts, n_order);

%% display the trajectory and cooridor

x_pos = [];
y_pos = [];
x_vel = [];
y_vel = [];
x_acc = [];
y_acc = [];
x_jerk = [];
y_jerk = [];
idx = 1;

%% #####################################################
% STEP 4: draw bezier curve
for j = 1:n_seg
    for t = 0:0.1:1
        x_pos(idx) = 0.0;y_pos(idx) = 0.0;
        x_vel(idx) = 0.0;y_vel(idx) = 0.0;
        x_acc(idx) = 0.0;y_acc(idx) = 0.0;
        x_jerk(idx) = 0.0;y_jerk(idx) = 0.0;
        for i = 0:n_order
            start_idx = (j-1)*(n_order+1)+i;
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);
            x_pos(idx) = x_pos(idx) + poly_coef_x(start_idx+1)*basis_p*ts(j);
            y_pos(idx) = y_pos(idx) + poly_coef_y(start_idx+1)*basis_p*ts(j);
            if i < n_order
                basis_v = nchoosek(n_order-1, i) * t^i *(1-t)^(n_order-1-i);
                x_vel(idx) = x_vel(idx) + (n_order+1) * (poly_coef_x(start_idx+2)-poly_coef_x(start_idx+1))*basis_v;
                y_vel(idx) = y_vel(idx) + (n_order+1) * (poly_coef_y(start_idx+2)-poly_coef_y(start_idx+1))*basis_v;
            end
            if i < n_order-1
                basis_a = nchoosek(n_order-2, i) * t^i *(1-t)^(n_order-2-i);
                x_acc(idx) = x_acc(idx) + (n_order+1) * n_order * (poly_coef_x(start_idx+3) - 2*poly_coef_x(start_idx+2) + poly_coef_x(start_idx+1))*basis_a/ts(j);
                y_acc(idx) = y_acc(idx) + (n_order+1) * n_order * (poly_coef_y(start_idx+3) - 2*poly_coef_y(start_idx+2) + poly_coef_y(start_idx+1))*basis_a/ts(j);
            end
            if i < n_order-2
                basis_j = nchoosek(n_order-3, i) * t^i *(1-t)^(n_order-3-i);
                x_jerk(idx) = x_jerk(idx) + (n_order+1) * n_order * (n_order-1) * ...
                    (poly_coef_x(start_idx+4) - 3*poly_coef_x(start_idx+3) + 3*poly_coef_x(start_idx+2) - poly_coef_x(start_idx+1)) * basis_j/ts(j)^2;
                y_jerk(idx) = y_jerk(idx) + (n_order+1) * n_order * (n_order-1) * ...
                    (poly_coef_y(start_idx+4) - 3*poly_coef_y(start_idx+3) + 3*poly_coef_y(start_idx+2) - poly_coef_y(start_idx+1)) * basis_j/ts(j)^2;
            end
        end
        idx = idx + 1;
    end
end
x = reshape(x_pos',[],1);
y = reshape(y_pos',[],1);
theta = [];
sz=size(x,1);
for i=1:sz-1
    if hypot(y(i+1)-y(i),x(i+1)-x(i))<0.01 && i>1
        theta = [theta;theta(i-1)];
    else
        theta = [theta;atan2(y(i+1)-y(i),x(i+1)-x(i))];
    end
end
theta = [theta;0];
x = x';
y = y';
theta = theta';

[x, y, theta] = ResamplePathWithEqualDistance(x, y, theta);
end


function poly_coef = MinimumSnapCorridorBezierSolver(axis, corridor, ts, n_order)
    %% #####################################################
    % STEP 1: compute Q_0 of c'Q_0c
    [Q, M]  = computeQM(n_order, ts);
    
    Q_0 = M'*Q*M;
    Q_0 = nearestSPD(Q_0);
    q = zeros(size(Q_0,1));
    %% #####################################################
    % STEP 2: get Aeq and beq
    [Aeq, beq] = getAbeq(n_order, ts, axis);
    %% #####################################################
    % STEP 3: get corridor_range, Aieq and bieq 
    
    % STEP 3.1: get corridor_range of x-axis or y-axis,
    % you can define corridor_range as [p1_min, p1_max;
    %                                   p2_min, p2_max;
    %                                   ...,
    %                                   pn_min, pn_max ];
    n_seg = size(ts,1);
    corridor_range = zeros(n_seg,2);
    for k = 1:n_seg
        corridor_range(k,:)=[corridor(k,2*axis-1),corridor(k,2*axis)];
    end
    [Aieq,bieq] = getAbieq(n_order, corridor_range, ts);
    
    % STEP 3.2: get Aieq and bieq
    
    poly_coef = cplexqp(Q_0/norm(Q_0,2),q,Aieq, bieq, Aeq, beq);
    if size(poly_coef ,1)==0
        error(num2str(axis)+" solve falied");
    else
        disp([num2str(axis), " solve succeed"]);
    end
end

function is_collision_free = Is3DNodeValid(child_node_config)
is_collision_free = 0;
global vehicle_geometrics_ planning_scale_ hybrid_astar_ costmap_
xr = child_node_config(:,1) + vehicle_geometrics_.r2x * cos(child_node_config(:,3));
yr = child_node_config(:,2) + vehicle_geometrics_.r2x * sin(child_node_config(:,3));
xf = child_node_config(:,1) + vehicle_geometrics_.f2x * cos(child_node_config(:,3));
yf = child_node_config(:,2) + vehicle_geometrics_.f2x * sin(child_node_config(:,3));
xx = [xr; xf];
yy = [yr; yf];
if (sum(xx > planning_scale_.xmax - vehicle_geometrics_.radius * 1.01))
    return;
elseif (sum(xx < planning_scale_.xmin + vehicle_geometrics_.radius * 1.01))
    return;
elseif (sum(yy > planning_scale_.ymax - vehicle_geometrics_.radius * 1.01))
    return;
elseif (sum(yy < planning_scale_.ymin + vehicle_geometrics_.radius * 1.01))
    return;
end
indx = round((xx - planning_scale_.xmin) /  hybrid_astar_.resolution_x + 1);
indy = round((yy - planning_scale_.ymin) /  hybrid_astar_.resolution_y + 1);
if (sum(costmap_(sub2ind(size(costmap_),indx,indy))))
    return;
end
is_collision_free = 1;
end

function is_collision_free = Is2DNodeValid(child_node_config, child_node_ind)
is_collision_free = 1;
global planning_scale_ costmap_
if (costmap_(child_node_ind(1), child_node_ind(2)) == 1)
    is_collision_free = 0;
    
    return;
end
if ((child_node_config(1) > planning_scale_.xmax) || (child_node_config(1) < planning_scale_.xmin) || (child_node_config(2) > planning_scale_.ymax) || (child_node_config(2) < planning_scale_.ymin))
    is_collision_free = 0;
    
    return;
end
end

function idx = Convert2DimConfigToIndex(config)
global hybrid_astar_ planning_scale_
ind1 = ceil((config(1) - planning_scale_.xmin) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((config(2) - planning_scale_.ymin) / hybrid_astar_.resolution_y) + 1;
idx = [ind1, ind2];
if ((ind1 <= hybrid_astar_.num_nodes_x)&&(ind1 >= 1)&&(ind2 <= hybrid_astar_.num_nodes_y)&&(ind2 >= 1))
    return;
end
if (ind1 > hybrid_astar_.num_nodes_x)
    ind1 = hybrid_astar_.num_nodes_x;
elseif (ind1 < 1)
    ind1 = 1;
end
if (ind2 > hybrid_astar_.num_nodes_y)
    ind2 = hybrid_astar_.num_nodes_y;
elseif (ind2 < 1)
    ind2 = 1;
end
idx = [ind1, ind2];
end

function costmap = CreateDilatedCostmap()
global planning_scale_ hybrid_astar_ vehicle_geometrics_ Nobs obstacles_
xmin = planning_scale_.xmin;
ymin = planning_scale_.ymin;
resolution_x = hybrid_astar_.resolution_x;
resolution_y = hybrid_astar_.resolution_y;
costmap = zeros(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y);

for ii = 1 : Nobs
    vx = obstacles_{ii}.x;
    vy = obstacles_{ii}.y;
    x_lb = min(vx); x_ub = max(vx); y_lb = min(vy); y_ub = max(vy);
    [Nmin_x,Nmin_y] = ConvertXYToIndex(x_lb,y_lb);
    [Nmax_x,Nmax_y] = ConvertXYToIndex(x_ub,y_ub);
    for jj = Nmin_x : Nmax_x
        for kk = Nmin_y : Nmax_y
            if (costmap(jj,kk) == 1)
                continue;
            end
            cur_x = xmin + (jj - 1) * resolution_x;
            cur_y = ymin + (kk - 1) * resolution_y;
            if (inpolygon(cur_x, cur_y, obstacles_{ii}.x, obstacles_{ii}.y) == 1)
                costmap(jj,kk) = 1;
            end
        end
    end
end
length_unit = 0.5 * (resolution_x + resolution_y);
basic_elem = strel('disk', ceil(vehicle_geometrics_.radius / length_unit));
costmap = imdilate(costmap, basic_elem);
end

function [ind1,ind2] = ConvertXYToIndex(x,y)
global hybrid_astar_ planning_scale_
ind1 = ceil((x - planning_scale_.xmin) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((y - planning_scale_.ymin) / hybrid_astar_.resolution_y) + 1;
if ((ind1 <= hybrid_astar_.num_nodes_x)&&(ind1 >= 1)&&(ind2 <= hybrid_astar_.num_nodes_y)&&(ind2 >= 1))
    return;
end
if (ind1 > hybrid_astar_.num_nodes_x)
    ind1 = hybrid_astar_.num_nodes_x;
elseif (ind1 < 1)
    ind1 = 1;
end
if (ind2 > hybrid_astar_.num_nodes_y)
    ind2 = hybrid_astar_.num_nodes_y;
elseif (ind2 < 1)
    ind2 = 1;
end
end

function [path] = Resample(path)
x = path(:,1);
y = path(:,2);
x_extended = [];
y_extended = [];
for ii = 1 : (length(x) - 1)
    distance = hypot(x(ii+1)-x(ii), y(ii+1)-y(ii));
    LARGE_NUM = round(distance/1.3);
    temp = linspace(x(ii), x(ii+1), LARGE_NUM);
    temp = temp(1,1:(LARGE_NUM - 1));
    x_extended = [x_extended, temp];
    
    temp = linspace(y(ii), y(ii+1), LARGE_NUM);
    temp = temp(1,1:(LARGE_NUM - 1));
    y_extended = [y_extended, temp];
end
x_extended = [x_extended, x(end)];
y_extended = [y_extended, y(end)];
x = x_extended';
y = y_extended';
path=[x,y];
% disp('path');
% disp(path);
end

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

function [x, y, theta] = ResamplePathWithEqualDistance(x, y, theta)
    for ii = 2 : length(theta)
        while (theta(ii) - theta(ii-1) > pi)
            theta(ii) = theta(ii) - 2 * pi;
        end
        while (theta(ii) - theta(ii-1) < -pi)
            theta(ii) = theta(ii) + 2 * pi;
        end
    end
    x_extended = [];
    y_extended = [];
    theta_extended = [];
    for ii = 1 : (length(x) - 1)
        distance = hypot(x(ii+1)-x(ii), y(ii+1)-y(ii));
        LARGE_NUM = round(distance * 100);
        temp = linspace(x(ii), x(ii+1), LARGE_NUM);
        temp = temp(1,1:(LARGE_NUM - 1));
        x_extended = [x_extended, temp];

        temp = linspace(y(ii), y(ii+1), LARGE_NUM);
        temp = temp(1,1:(LARGE_NUM - 1));
        y_extended = [y_extended, temp];

        temp = linspace(theta(ii), theta(ii+1), LARGE_NUM);
        temp = temp(1,1:(LARGE_NUM - 1));
        theta_extended = [theta_extended, temp];
    end
    x_extended = [x_extended, x(end)];
    y_extended = [y_extended, y(end)];
    theta_extended = [theta_extended, theta(end)];
    global num_nodes_s
    index = round(linspace(1, length(x_extended), num_nodes_s));
    x = x_extended(index);
    y = y_extended(index);
    theta = theta_extended(index);
end
