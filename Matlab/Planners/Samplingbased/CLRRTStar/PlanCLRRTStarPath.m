function [x, y, theta, path_length, completeness_flag] = PlanCLRRTStarPath()
    CL_RRTStarSetup()
    global costmap_
    costmap_ = CreateDilatedCostmap();
    global CL_RRTStar_
    CL_RRTStar_.node_num = 1;
    CL_RRTStar_.node_list(:, CL_RRTStar_.node_num) = [CL_RRTStar_.start, CL_RRTStar_.start(1:2), 0, 0, 1, 1];
    % build the rrt-tree
    
    while 1
        random_sample = Get_Random_Sample();
        nearest_node_idx = Find_Nearest_Node(random_sample);
        node_new_idx = Create_New_Node(nearest_node_idx, random_sample); 
        if (node_new_idx == 0)
            continue;
        end
        
        Rewire(node_new_idx);
        
        if(Is_Goal(node_new_idx))
            break
        end
    end

    % get traj
    traj = Get_Traj(node_new_idx); % traj  4 x n ([x, y, v, theta])
    x = traj(1,:);
    y = traj(2,:);
    theta = traj(3,:);
    path_length = 0;
    for i = 1 : length(x) - 1
        path_length = path_length + hypot(x(i+1) - x(i), y(i+1) - y(i));
    end
    completeness_flag = 1;
    
    return;
end

function costmap = CreateDilatedCostmap()
    global planning_scale_ CL_RRTStar_ vehicle_geometrics_ Nobs obstacles_
    xmin = planning_scale_.xmin;
    ymin = planning_scale_.ymin;
    resolution_x = CL_RRTStar_.resolution_x;
    resolution_y = CL_RRTStar_.resolution_y;
    costmap = zeros(CL_RRTStar_.grid_num_x, CL_RRTStar_.grid_num_y);

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
    global CL_RRTStar_ planning_scale_
    ind1 = ceil((x - planning_scale_.xmin) / CL_RRTStar_.resolution_x) + 1;
    ind2 = ceil((y - planning_scale_.ymin) / CL_RRTStar_.resolution_y) + 1;
    if ((ind1 <= CL_RRTStar_.grid_num_x)&&(ind1 >= 1)&&(ind2 <= CL_RRTStar_.grid_num_y)&&(ind2 >= 1))
        return;
    end
    if (ind1 > CL_RRTStar_.grid_num_x)
        ind1 = CL_RRTStar_.grid_num_x;
    elseif (ind1 < 1)
        ind1 = 1;
    end
    if (ind2 > CL_RRTStar_.grid_num_y)
        ind2 = CL_RRTStar_.grid_num_y;
    elseif (ind2 < 1)
        ind2 = 1;
    end
end

function CL_RRTStarSetup()
    global CL_RRTStar_ planning_scale_ vehicle_TPBV_ vehicle_kinematics_
    CL_RRTStar_.start = [vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0, vehicle_TPBV_.v0];
    CL_RRTStar_.goal = [vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf, vehicle_TPBV_.vtf];
    CL_RRTStar_.resolution_x = 0.5;
    CL_RRTStar_.resolution_y = 0.5;
    CL_RRTStar_.sim_timestep = 0.5;
    CL_RRTStar_.expand_dis = 15;
    CL_RRTStar_.goal_sample_rate = 0;
    CL_RRTStar_.connect_circle_dist = 15;
    CL_RRTStar_.loc_threshold = 1.0;
    CL_RRTStar_.node_num = 0;
    CL_RRTStar_.node_list = zeros(10, 500, 'double');
    % Information of each element in each node:
    % Dim % | Variable
    %  1        x
    %  2        y
    %  3        theta
    %  4        v
    %  5        cmd_x
    %  6        cmd_y
    %  7        cost
    %  8        parent_idx
    %  9        reachable
    %  10       terminate
    CL_RRTStar_.grid_num_x = ceil(planning_scale_.x_scale / CL_RRTStar_.resolution_x);
    CL_RRTStar_.grid_num_y = ceil(planning_scale_.y_scale / CL_RRTStar_.resolution_y);
    CL_RRTStar_.L_max = 3 * (vehicle_kinematics_.vehicle_v_max < 1.34) + 2.24 * vehicle_kinematics_.vehicle_v_max * (vehicle_kinematics_.vehicle_v_max >= 1.34 && vehicle_kinematics_.vehicle_v_max < 5.36) + 12 * (vehicle_kinematics_.vehicle_v_max >= 5.36);

end

function random_sample = Get_Random_Sample()
    %Get a random sample in controller input space
    global CL_RRTStar_ planning_scale_
    %tolerance for sample point to exceed planning scale (since sample point may not be reached considering the lookforward dis in pure_persuit)
    tolerance = 3;
    if (rand() < CL_RRTStar_.goal_sample_rate)
        random_sample = [CL_RRTStar_.goal(1), CL_RRTStar_.goal(2)];
    else
        x = planning_scale_.xmin - tolerance + (planning_scale_.x_scale + 2 * tolerance) * rand();
        y = planning_scale_.ymin - tolerance + (planning_scale_.y_scale + 2 * tolerance) * rand();
        random_sample = [x, y];
    end
end

function idx = Create_New_Node(sorted_idxs, random_sample)
%     """
%     Create node_new
%     Args:
%         random_sample (tuple): [x, y]
%     """
    global CL_RRTStar_ planning_scale_
    idx = 0;
    for node_parent_idx = sorted_idxs
        if (CL_RRTStar_.node_list(9, node_parent_idx) == 0)
            % unreachable
            continue;
        end
        dx = random_sample(1) - CL_RRTStar_.node_list(5, node_parent_idx);
        dy = random_sample(2) - CL_RRTStar_.node_list(6, node_parent_idx);
        norm = hypot(dx, dy);
        if(norm == 0)
            continue;
        end
        
        dis_to_goal = hypot(CL_RRTStar_.node_list(1, node_parent_idx) - CL_RRTStar_.goal(1), CL_RRTStar_.node_list(2, node_parent_idx) - CL_RRTStar_.goal(2));
        expand_dis = min(CL_RRTStar_.expand_dis, dis_to_goal);
        
        new_x = CL_RRTStar_.node_list(5, node_parent_idx) + dx * expand_dis / norm;
        new_y = CL_RRTStar_.node_list(6, node_parent_idx) + dy * expand_dis / norm;
        new_x = min(new_x, planning_scale_.xmax);
        new_x = max(new_x, planning_scale_.xmin);
        new_y = min(new_y, planning_scale_.ymax);
        new_y = max(new_y, planning_scale_.ymin);

        node_new = [0, 0, 0, 0, new_x, new_y, 0, 0, 0, 0];
        % decide whether or not the car should stop at the new point
        if(hypot(new_x - CL_RRTStar_.goal(1), new_y - CL_RRTStar_.goal(2)) <= 5)
            node_new(10) = 1;
        end
        
        [feasible, traj, path_length] = Propagate(CL_RRTStar_.node_list(:, node_parent_idx), node_new);
        if (feasible)
            node_new(8) = node_parent_idx;
            node_new(9) = 1;
            node_new(1:4) = traj(:, end);
            node_new(7) = CL_RRTStar_.node_list(7, node_parent_idx) + path_length;
            CL_RRTStar_.node_num = CL_RRTStar_.node_num + 1;
            CL_RRTStar_.node_list(:,CL_RRTStar_.node_num) = node_new;
            idx = CL_RRTStar_.node_num;
            return;
        end
    end
end

function idx = Find_Nearest_Node(random_sample)
%         """
%         find the nearest node to the random_sample given
% 
%         Args:
%             random_sample(tuple): (x, y)
%         
%         Returns:
%             int: indexs of node in sorted order
%         """
    global CL_RRTStar_
    dis = zeros(1, CL_RRTStar_.node_num);
    for i = 1 : CL_RRTStar_.node_num
        if CL_RRTStar_.node_list(9, i) > 0
            dis(i) = Dubins_Dis(CL_RRTStar_.node_list(:,i), random_sample);
        else
            dis(i) = 10000;
        end
    end
    [~, idx] = min(dis);
    return;
end

function Rewire(node_new_idx)
%     """
%     Rewire the given node and the nodes near it. If sucess, append the node to the tree
%     """
    global CL_RRTStar_
    near_node_idx = Find_Near_Nodes(node_new_idx);
    is_reparent = false;
    % calculate distance from near_nodes to node_new
    dis = zeros(1, length(near_node_idx));
    for i = 1 : length(near_node_idx)
        dis(i) = Dubins_Dis(CL_RRTStar_.node_list(:, near_node_idx(i)), CL_RRTStar_.node_list(1:2, node_new_idx));
    end

    % find a new parent for node_new
    i = 1;
    for idx = near_node_idx
        if (dis(i) + CL_RRTStar_.node_list(7,idx) < CL_RRTStar_.node_list(7,node_new_idx) && CL_RRTStar_.node_list(9,idx) > 0)
            % check feasibility
            [feasible , traj, path_length] = Propagate(CL_RRTStar_.node_list(:,idx), CL_RRTStar_.node_list(:,node_new_idx));
            if (feasible)
                CL_RRTStar_.node_list(8, node_new_idx) = idx;
                CL_RRTStar_.node_list(7,node_new_idx) = path_length + CL_RRTStar_.node_list(7, idx);
                CL_RRTStar_.node_list(1:4,node_new_idx) = traj(:, end);
            end
        end
        i = i + 1;
    end
    % rewire other nodes
    i = 1;
    for idx = near_node_idx
        dis(i) = Dubins_Dis(CL_RRTStar_.node_list(:,node_new_idx), CL_RRTStar_.node_list(1:2, near_node_idx(i)));
        if(dis(i) + CL_RRTStar_.node_list(7, node_new_idx) < CL_RRTStar_.node_list(7, idx))
            [feasible, traj, path_length] = Propagate(CL_RRTStar_.node_list(:, node_new_idx), CL_RRTStar_.node_list(:, idx));
            if(feasible)
                CL_RRTStar_.node_list(8, idx) = node_new_idx;
                CL_RRTStar_.node_list(7, idx) = path_length + CL_RRTStar_.node_list(7, node_new_idx);
                CL_RRTStar_.node_list(1:4, idx) = traj(:, end);
                is_reparent = true;
            end
        end
        i = i + 1;
    end
    % update the tree
    if (is_reparent)
        Update_Tree(node_new_idx);
    end
end

function flag = Is_Goal(node_idx)
    global CL_RRTStar_
    dx = CL_RRTStar_.goal(1) - CL_RRTStar_.node_list(1, node_idx);
    dy = CL_RRTStar_.goal(2) - CL_RRTStar_.node_list(2, node_idx);
    if hypot(dx, dy) <= CL_RRTStar_.loc_threshold
        flag = (CL_RRTStar_.node_list(9, node_idx) > 0) & (CL_RRTStar_.node_list(10, node_idx) == 1);
        return;
    end
    flag = false;
end

function [feasible, traject, path_length] = Propagate(node_from, node_to)
%     """
%     Propagate form node_from to node_to, using the control law describe in KuwataGNC08
%     """
    global vehicle_kinematics_ vehicle_geometrics_ CL_RRTStar_ planning_scale_
    t_min = 0.5;
    % form the controller input(draw a line)
    x_step = (node_to(5) - node_from(5)) / 49;
    y_step = (node_to(6) - node_from(6)) / 49;
    c_x = zeros(1, 50);
    c_y = zeros(1, 50);
    for i = 1:50
        c_x(i) = node_from(5) + x_step * (i - 1);
        c_y(i) = node_from(6) + y_step * (i - 1);
    end
    % get the initial look-ahead point
    idx = Get_Look_Ahead_Point_Idx(c_x, c_y, node_from(1:4));
    % generate v_profile
    % params for calculating v_coast
    a_accel = abs(vehicle_kinematics_.vehicle_a_max);
    a_decel = abs(vehicle_kinematics_.vehicle_a_min);
    alpha_2 = -0.0252;
    alpha_1 = 1.2344;
    alpha_0 = -0.5347;
    % calculate v_coast
    v = abs(node_from(4));
    L_1 = 3 * (node_from(4) < 1.34) + 2.24 * node_from(4) * (node_from(4) >= 1.34 && node_from(4) < 5.36) + 12 * (node_from(4) >= 5.36);
    if(node_to(10) == 1)
        L_min = 3;
    else
        L_min = CL_RRTStar_.L_max;
    end
    D = L_1 + hypot(c_x(end) - c_x(idx), c_y(end) - c_x(idx)) - L_min;
    f = @(v) v^2 / (2 * a_decel) + alpha_2 * v^2 + alpha_1 * v + alpha_0;
    pi_to_pi = @(angle) mod(angle + pi, 2 * pi) - pi;
    
    if(node_to(10) == 1)
        % vehicle stops at the end
        if D > (vehicle_kinematics_.vehicle_v_max^2 - node_from(4) ^ 2) / (2 * vehicle_kinematics_.vehicle_a_max) + vehicle_kinematics_.vehicle_v_max * t_min + f(vehicle_kinematics_.vehicle_v_max)
            v_coast = vehicle_kinematics_.vehicle_v_max;
        else
            A = 1 / (2*a_accel) + 1 / (2*a_decel) + alpha_2;
            B = t_min + alpha_1;
            C = alpha_0 - D - node_from(4) ^ 2 / (2 * a_accel);
            v_coast = (-1 * B + sqrt(B ^ 2 - 4 * A * C)) / (2 * A);
        end
        v_ramp_up = node_from(4): a_accel * CL_RRTStar_.sim_timestep :v_coast;
        v_coasting = ones([1, floor(t_min / CL_RRTStar_.sim_timestep)]) * v_coast;
        v_ramp_down = v_coast: -a_decel * CL_RRTStar_.sim_timestep: 0;
        if(v_ramp_down(end) ~= 0)
            v_ramp_down(end+1) = 0;
        end
        v_profile = [v_ramp_up, v_coasting, v_ramp_down];
    else
        % vehicle does not stop at the end
        v_coast = vehicle_kinematics_.vehicle_v_max;
        v_ramp_up = node_from(4): a_accel * CL_RRTStar_.sim_timestep :v_coast;
        D = D - (v_coast ^ 2 - node_from(4) ^ 2) / 2 / a_accel;
        v_coasting = ones([1, floor(D / CL_RRTStar_.sim_timestep / v_coast)]) * v_coast;
        v_profile = [v_ramp_up, v_coasting];
    end

    q_t = node_from(1:4); %% temporary state
    
    traject = zeros(4, length(v_profile) - 1);
    traject (:, 1) = q_t;
    idx = Get_Look_Ahead_Point_Idx(c_x, c_y, q_t);
    
    % steering control
    eta = q_t(3) - atan2((c_y(idx) - q_t(2)), (c_x(idx) - q_t(1)));
    eta = pi_to_pi(eta);

    if(abs(eta) < pi / 2)
        move_dir = 1; % forward
    else
        move_dir = -1; % backward
    end
    i = 1;
    path_length = 0;
    for v = v_profile
        if v == 0
            continue
        end
        idx = Get_Look_Ahead_Point_Idx(c_x, c_y, q_t);
        % steering control
        Lf = 3 * (v < 1.34) + 2.24 * v * (v >= 1.34 & v < 5.36) + 12 * (v >= 5.36);
        eta = q_t(3) - atan2((c_y(idx) - q_t(2)), (c_x(idx) - q_t(1)));
        eta = pi_to_pi(eta);

        if(abs(eta) < pi / 2)
            delta = -1 * atan2(2.0 * vehicle_geometrics_.vehicle_length * sin(eta) / Lf, 1.0);
        else
            delta = -1 * atan2(2.0 * vehicle_geometrics_.vehicle_length * sin(pi - eta) / Lf, 1.0);
        end
        if (delta > vehicle_kinematics_.vehicle_phi_max)
            delta = vehicle_kinematics_.vehicle_phi_max;
        end
        if (delta < vehicle_kinematics_.vehicle_phi_min)
            delta = vehicle_kinematics_.vehicle_phi_min;
        end

        % update state
        path_length = path_length + v * CL_RRTStar_.sim_timestep;
        q_t(1) = q_t(1) + v * CL_RRTStar_.sim_timestep * cos(q_t(3)) * move_dir;
        q_t(2) = q_t(2) + v * CL_RRTStar_.sim_timestep * sin(q_t(3)) * move_dir;
        q_t(4) = (v * move_dir);
        q_t(3) = q_t(3) + (v * move_dir) / vehicle_geometrics_.vehicle_length * tan(delta);
        q_t(3) = pi_to_pi(q_t(3));
        % check feasibility
        if((q_t(1) < planning_scale_.xmin) || (q_t(1) > planning_scale_.xmax) ...
            || (q_t(2) < planning_scale_.ymin) || (q_t(2) > planning_scale_.ymax))
            feasible = false;
            return;
        end
        if(~ Check_Collision(q_t))
            feasible = false;
            return;
        end
        i = i + 1;
        traject(:, i) = q_t;
    end
    feasible = true;
    return;
end

function idx = Get_Look_Ahead_Point_Idx(c_x, c_y, q)
    v = abs(q(4));
    Lf = 3 * (v < 1.34) + 2.24 * v * (v >= 1.34 && v < 5.36) + 12 * (v >= 5.36);
    dx = q(1) - c_x;
    dy = q(2) - c_y;
    dis = zeros(1, 50);
    for i = 1:50
        dis(i) = hypot(dx(i), dy(i));
    end
    [~, min_idx] = min(dis);
    idx = length(dis);
    for i = min_idx : length(dis)
        if(dis(i) >= Lf)
            idx = i;
            return;
        end
    end
    return;
end
    
function flag = Check_Collision(q)
    global costmap_
    [idx_x, idx_y] = ConvertXYToIndex(q(1), q(2));
    flag = (costmap_(idx_x, idx_y) == 0);
    return;
end

function idxs = Find_Near_Nodes(node_new_idx)
    global CL_RRTStar_
    dis = zeros(1, CL_RRTStar_.node_num);
    for i = 1:CL_RRTStar_.node_num
        dis(i) = Dubins_Dis(CL_RRTStar_.node_list(:, i),  CL_RRTStar_.node_list(1:2,node_new_idx));
    end
    idxs = find(dis <= CL_RRTStar_.connect_circle_dist);
    return;
end


function traj = Get_Traj(goal_node_idx)
    global CL_RRTStar_
    path = goal_node_idx;
    parent = CL_RRTStar_.node_list(8, goal_node_idx);
    while (parent > 0)
        path(end + 1) = parent;
        parent = CL_RRTStar_.node_list(8, parent);
    end
    path = flip(path);
    
    traj = [];
    traj(:, 1) = CL_RRTStar_.start;
    for i = 1:length(path) - 1
        [~, traj_temp, ~] = Propagate(CL_RRTStar_.node_list(:, path(i)), CL_RRTStar_.node_list(:, path(i + 1)));
        traj = cat(2, traj, traj_temp(:, 2:end));
    end
    return;
end

function Update_Tree(parent_idx)
    global CL_RRTStar_
    for idx = 1: CL_RRTStar_.node_num
        node = CL_RRTStar_.node_list(:, idx);
        if (CL_RRTStar_.node_list(8, idx) == parent_idx)
            if(CL_RRTStar_.node_list(9, parent_idx) > 0)
                [feasable, traj, path_length] = Propagate(CL_RRTStar_.node_list(:,parent_idx), node);
                if(feasable)
                    CL_RRTStar_.node_list(7, idx) = path_length + CL_RRTStar_.node_list(7, parent_idx);
                    CL_RRTStar_.node_list(1:4, idx) = traj(:, end);
                    CL_RRTStar_.node_list(9, idx) = 1;
                else
                    % denote node as unreachable
                    CL_RRTStar_.node_list(9, idx) = 0;
                end
            else
                CL_RRTStar_.node_list(9, idx) = 0;
            end
            Update_Tree(idx)
        end
    end
end

function dubins_dis = Dubins_Dis(node_from, cmd_to)
    x = node_from(1);
    y = node_from(2);
    theta = node_from(3);
    R = [cos(theta) -sin(-theta); sin(-theta) cos(theta)];
    sxx = cmd_to(1) - x;
    syy = cmd_to(2) - y;
    s = R*[sxx; syy];
    sx_rel = s(1);
    sy_rel = abs(s(2));
    global vehicle_kinematics_
    rho = vehicle_kinematics_.min_turning_radius;
    if  sqrt((sx_rel - 0)^2 + (sy_rel - rho)^2) < rho %relative point is in Dp  - RTMP 1109
        df = sqrt((sx_rel^2) + (sy_rel + rho)^2);
        dc = sqrt((sx_rel^2) + (sy_rel - rho)^2);
        theta_i = atan2(sx_rel, rho - sy_rel);
        theta_c = mod((theta_i + 2*pi), 2*pi);
        phi = acos((5*rho^2 - df^2)/(4*rho^2));
        alpha = 2*pi - phi;
        dubins_dis = rho*(alpha + asin(dc*sin(theta_c)/df) + asin((rho*sin(phi))/df));
    else
        dc = sqrt((sx_rel^2) + (sy_rel - rho)^2);
        theta_i = atan2(sx_rel, rho - sy_rel);
        theta_c = mod((theta_i + 2*pi), 2*pi);
        dubins_dis = sqrt(dc^2 - rho^2) + rho*(theta_c - acos(rho/dc));
    end
end