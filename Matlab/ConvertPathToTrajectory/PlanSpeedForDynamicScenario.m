function trajectory = PlanSpeedForDynamicScenario(x, y, theta)
%% it includes three parts：a. search velolcity in St-Graph; b. optimize velolcity in St-Graph; c. transform s-t to trajectory.v;
tic;
[t0,s0] = SearchVelocityInStGraph(x, y, theta);
toc;
% figure;
% plot(t0,s0);
tic;
[t,s] = OptimizeVelocityInStGraph(t0,s0);
toc;
% figure;
% plot(t,s);
trajectory = TransformPathToTrajectoryForDynamicScenarios(x, y, theta, t, s);
end


function [t, s] = SearchVelocityInStGraph(x, y, theta)
global st_graph_search_ costmap_
costmap_ = zeros(st_graph_search_.num_nodes_t, st_graph_search_.num_nodes_s);
global dynamic_obs
figure(2)
for ii = 1 : st_graph_search_.num_nodes_t
    for jj = 1 : st_graph_search_.num_nodes_s
        cur_x = x(jj);
        cur_y = y(jj);
        cur_theta = theta(jj);
        for kk = 1 : size(dynamic_obs, 2)
            if (IsVehicleCollidingWithMovingObstacle(cur_x, cur_y, cur_theta, dynamic_obs{ii,kk}))
                costmap_(ii, jj) = 1;
                plot(ii,jj,'kx');hold on
                continue;
            end
        end
    end
end
axis([1 st_graph_search_.num_nodes_t 1 st_graph_search_.num_nodes_s]); grid on; box on;

ind_vec = SearchStPathViaAStar();
ind1 = ind_vec(:,1)';
ind2 = ind_vec(:,2)';
ind1n = ind1(1); ind2n = ind2(1);
for ii = 2 : length(ind1)
    if (ind1(ii) ~= ind1(ii-1))
        ind1n = [ind1n, ind1(ii)];
        ind2n = [ind2n, ind2(ii)];
    end
end
t = (ind1n - 1) .* st_graph_search_.resolution_t;
s = (ind2n - 1) .* st_graph_search_.resolution_s;
plot(ind_vec(:,1),ind_vec(:,2),'r');hold on; drawnow;
end

function ind_vec = SearchStPathViaAStar()
global st_graph_search_ costmap_
grid_space_2D_ = cell(st_graph_search_.num_nodes_t, st_graph_search_.num_nodes_s);
init_node = zeros(1,11);
goal_ind = [st_graph_search_.num_nodes_t, st_graph_search_.num_nodes_s];
% Information of each element in each node:
% Dim # | Variable
%  1        null
%  2        null
%  3        f
%  4        g
%  5        h
%  6        is_in_openlist
%  7        is_in_closedlist
%  8-9      index of current node
%  10-11    index of parent node
init_node(4) = 0;
init_node(6) = 1;
init_node(8:9) = [1, 1];
init_node(5) = sum(abs(init_node(8:9) - goal_ind));
init_node(3) = init_node(4) + st_graph_search_.multiplier_H_for_A_star * init_node(5);
init_node(10:11) = [-999,-999];
openlist_ = init_node;
grid_space_2D_{init_node(8), init_node(9)} = init_node;
% % We choose 5 neiborhood expansion, which indicates that we enable that s
% may NOT change monotonously
expansion_pattern = [
    0 1;
    0 -1;
    1 1;
    1 -1;
    1 0];
expansion_length = [
    1 + st_graph_search_.penalty_for_inf_velocity;
    1 + st_graph_search_.penalty_for_inf_velocity;
    1.414;
    1.414;
    1];
completeness_flag = 0;

iter = 0;
while (~isempty(openlist_))
    iter = iter + 1;
    cur_node_order = find(openlist_(:,3) == min(openlist_(:,3))); cur_node_order = cur_node_order(end);
    cur_node = openlist_(cur_node_order, :);
    cur_ind = cur_node(8:9);
    if ((cur_ind(1) == goal_ind(1))&&(cur_ind(2) == goal_ind(2)))
        completeness_flag = 1;
        break;
    end
    cur_g = cur_node(4);
    % Remove cur_node from open list and add it in closed list
    openlist_(cur_node_order, :) = [];
    grid_space_2D_{cur_ind(1), cur_ind(2)}(6) = 0;
    grid_space_2D_{cur_ind(1), cur_ind(2)}(7) = 1;
    for ii = 1 : size(expansion_pattern,1)
        child_node_ind = cur_ind + expansion_pattern(ii,:);
        if ((child_node_ind(1) < 1)||(child_node_ind(1) > st_graph_search_.num_nodes_t)||(child_node_ind(2) < 1)||(child_node_ind(2) > st_graph_search_.num_nodes_s))
            continue;
        end
        % If the child node has been explored ever before, and then if the child has been within the closed list, abandon it and continue.
        if ((~isempty(grid_space_2D_{child_node_ind(1), child_node_ind(2)}))&&(grid_space_2D_{child_node_ind(1), child_node_ind(2)}(7) == 1))
            continue;
        end
        child_g = cur_g + expansion_length(ii);
        child_h = sum(abs(child_node_ind - goal_ind));
        child_f = child_g + st_graph_search_.multiplier_H_for_A_star * child_h;
        child_node_prepare = [0, 0, child_f, child_g, child_h, 1, 0, child_node_ind, cur_ind];
        % If the child node has been explored ever before
        if (~isempty(grid_space_2D_{child_node_ind(1), child_node_ind(2)}))
            % The child must be in the open list now, then check if its
            % recorded parent deserves to be switched as our cur_node.
            if (grid_space_2D_{child_node_ind(1), child_node_ind(2)}(4) > child_g + 0.1)
                child_node_order1 = find(openlist_(:,8) == child_node_ind(1));
                child_node_order2 = find(openlist_(child_node_order1,9) == child_node_ind(2));
                openlist_(child_node_order1(child_node_order2), :) = [];
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
                openlist_ = [openlist_; child_node_prepare];
            end
        else % Child node has never been explored before
            % If the child node is collison free
            if (costmap_(child_node_ind(1),child_node_ind(2)) == 0)
                openlist_ = [openlist_; child_node_prepare];
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
            else % If the child node involves collisons
                child_node_prepare(7) = 1;
                child_node_prepare(6) = 0;
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
            end
        end
    end
end

if (completeness_flag)
    ind_vec = cur_node(8:9);
else
    ind_vec = [];
    return;
end
parent_ind = grid_space_2D_{ind_vec(1), ind_vec(2)}(10:11);
while (parent_ind(1) > -1)
    ind_vec = [parent_ind; ind_vec];
    parent_ind = grid_space_2D_{parent_ind(1), parent_ind(2)}(10:11);
end
end

function is_collided = IsVehicleCollidingWithMovingObstacle(x, y, theta, V)
is_collided = 0;
if (min(hypot(V.x - x, V.y - y)) > 10)
    return;
end
Vcar = CreateVehiclePolygonFull(x, y, theta);
if (any(inpolygon(Vcar.x, Vcar.y, V.x, V.y)))
    is_collided = 1;
    return;
end
if (any(inpolygon(V.x, V.y, Vcar.x, Vcar.y)))
    is_collided = 1;
    return;
end
end

function Vcar = CreateVehiclePolygonFull(x, y, theta)
global vehicle_geometrics_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;
AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;
CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
Vcar.x = [AX, BX, CX, DX, AX];
Vcar.y = [AY, BY, CY, DY, AY];
end

function [t, s] = OptimizeVelocityInStGraph(t0,s0)
global st_graph_search_ vehicle_kinematics_ 
delta_t = st_graph_search_.resolution_t;
% index_t = find(s0==max(s0),1); s0=s0(1:index_t); t0=t0(1:index_t); %考虑到在原估计的大概时间前已规划到了路径的终点，因此去除在终点的等待时间
t = t0;
nstep = length(s0);
s0 = s0'; %转为列向量
%% The cost function
% The distance metric between the original path and the new path
Q1 = eye(nstep);
% The velocity
Vdiff = eye(nstep)-diag(ones(1,(nstep-1)),1);
Vdiff = delta_t*Vdiff;
Q2 = Vdiff(1:(nstep-1),:)'*Q1(2:end,1+1:end)*Vdiff(1:(nstep-1),:);
% The accelaration
Adiff = eye(nstep)-2*diag(ones(1,(nstep-1)),1)+diag(ones(1,(nstep-2)),2);
Adiff = (delta_t)^2*Adiff;
Q3 = Adiff(1:(nstep-2),:)'*Adiff(1:(nstep-2),:);
% The jerk
Jdiff = eye(nstep)-3*diag(ones(1,(nstep-1)),1)+3*diag(ones(1,(nstep-2)),2)-diag(ones(1,(nstep-3)),3);
Jdiff = (delta_t)^3*Jdiff;
Q4 = Jdiff(1:(nstep-2),:)'*Jdiff(1:(nstep-2),:);
% The weight
cref = [1,0,0,0];
cabs = [0,1,1,1];
% The total costj
Qref = Q1*cref(1)+Q2*cref(2)+Q3*cref(3)+Q4*cref(4);
Qabs = Q1*cabs(1)+Q2*cabs(2)+Q3*cabs(3)+Q4*cabs(4);

%% The initial and terminal position constraint
Aeq = zeros(2,nstep);
Aeq(1,1) = 1; Aeq(2,end) = 1;
beq = [s0(1);s0(end)];

%% s bound limit
lb = zeros(nstep,1);
ub = max(s0)*ones(nstep,1);

%% Speed limit
Aineq = [Vdiff(1:(nstep-1),:);-Vdiff(1:(nstep-1),:)];
v_max = vehicle_kinematics_.vehicle_v_max; v_min = vehicle_kinematics_.vehicle_v_min;
bineq = [v_max*ones(nstep-1,1);-v_min*ones(nstep-1,1)]; 

H = Qref+Qabs;
f = -Qref*s0;
s = cplexqp(H,f,Aineq,bineq,Aeq,beq,lb,ub,s0); % 1/2*x'*H*x + f*x
 
if isempty(s)
    disp('The problem is unfeasible. Please check the constraints');
end
end


function trajectory = TransformPathToTrajectoryForDynamicScenarios(x, y, theta, t, s)
global st_graph_search_ vehicle_kinematics_ vehicle_geometrics_
delta_t = st_graph_search_.resolution_t;
if ( length( t ) > st_graph_search_.num_nodes_t )
    tt = t( end  );
    ss = s( end  );
    for ii = ( length( t ) - 1 ): - 1:1
        if ( t( ii + 1 ) ~= t( ii ) )
            tt = [ t( ii ), tt ];
            ss = [ s( ii ), ss ];
        end
    end
    tt( 1 ) = 0;ss( 1 ) = 0;
    t = tt;s = ss;
end
if ( length( t ) ~= st_graph_search_.num_nodes_t )
    error '[DemonstrateDynamicResult] Error code 1';
end
[ x, y, theta ] = ResamplePathWithEqualDistance( x, y, theta );
%% 按单位时间所对应的s重新分配x,y,theta的坐标
ss = zeros( 1, length( x ) );
for ii = 2:length( x )
    ss( ii ) = ss( ii - 1 ) + hypot( x( ii ) - x( ii - 1 ), y( ii ) - y( ii - 1 ) );
end
trajectory.x = [  ];
trajectory.y = [  ];
trajectory.theta = [  ];
for ii = 1:length( s )
    err = abs( s( ii ) - ss );
    ind = find( err == min( err ) );ind = ind( end  );
    trajectory.x = [ trajectory.x, x( ind ) ];
    trajectory.y = [ trajectory.y, y( ind ) ];
    trajectory.theta = [ trajectory.theta, theta( ind ) ];
end
%% 根据新x,y,theta求轨迹的其他变量
Nfe = length(trajectory.x);
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
dt = delta_t; %  tmax/  Nfe;   %path_length / Nfe;
for ii = 2 : Nfe
    v(ii) = vdr(ii) * sqrt(((x(ii) - x(ii-1)) / dt)^2 + ((y(ii) - y(ii-1)) / dt)^2);
end
for ii = 2 : Nfe
    a(ii) = (v(ii) - v(ii-1)) / dt;
end
phi = zeros(1, Nfe);
omega = zeros(1, Nfe);
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
trajectory.v = v; trajectory.a = a;  trajectory.phi = phi; trajectory.omega = omega;  

end

function [ x, y, theta ] = ResamplePathWithEqualDistance( x, y, theta )
for ii = 2:length( theta )
    while ( theta( ii ) - theta( ii - 1 ) > pi )
        theta( ii ) = theta( ii ) - 2 * pi;
    end
    while ( theta( ii ) - theta( ii - 1 ) <  - pi )
        theta( ii ) = theta( ii ) + 2 * pi;
    end
end
x_extended = [  ];
y_extended = [  ];
theta_extended = [  ];
for ii = 1:( length( x ) - 1 )
    distance = hypot( x( ii + 1 ) - x( ii ), y( ii + 1 ) - y( ii ) );
    LARGE_NUM = round( distance * 100 );
    temp = linspace( x( ii ), x( ii + 1 ), LARGE_NUM );
    temp = temp( 1, 1:( LARGE_NUM - 1 ) );
    x_extended = [ x_extended, temp ];
    
    temp = linspace( y( ii ), y( ii + 1 ), LARGE_NUM );
    temp = temp( 1, 1:( LARGE_NUM - 1 ) );
    y_extended = [ y_extended, temp ];
    
    temp = linspace( theta( ii ), theta( ii + 1 ), LARGE_NUM );
    temp = temp( 1, 1:( LARGE_NUM - 1 ) );
    theta_extended = [ theta_extended, temp ];
end
x_extended = [ x_extended, x( end  ) ];
y_extended = [ y_extended, y( end  ) ];
theta_extended = [ theta_extended, theta( end  ) ];
index = round( linspace( 1, length( x_extended ), 1000 ) );
x = x_extended( index );
y = y_extended( index );
theta = theta_extended( index );
end
