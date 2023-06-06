function [x, y, theta, path_length, is_complete, time_consumed2] = PlanImprovedLatticePath(loc2pos_scale, lut_file, interpolation_file)
    %ImprovedLatticePlan Plan the path using Lattice planning method
    %   loc2pos_scale: the scale between real location and relative
    %   position in graph.
    %   lut_file: the lookup table filename.
    %   interpolation_file: the interpolation filename.
    global planning_scale_;
    global vehicle_TPBV_;
    graph_width = planning_scale_.x_scale;
    graph_height = planning_scale_.y_scale;
    action_set = ActionSet();
    action_set = action_set.load_data(interpolation_file);
    graph = StateLatticeGraph(int32(graph_width/loc2pos_scale), int32(graph_height / loc2pos_scale), action_set);

    start_loc = [vehicle_TPBV_.x0, vehicle_TPBV_.y0];
    start_direction = theta2direction(vehicle_TPBV_.theta0);
    end_loc = [vehicle_TPBV_.xtf, vehicle_TPBV_.ytf];
    end_direction = theta2direction(vehicle_TPBV_.thetatf);

    start_pos = loc2pos(start_loc, loc2pos_scale);
    end_pos = loc2pos(end_loc, loc2pos_scale);
    start_index = graph.position2index(start_pos, start_direction);
    end_index = graph.position2index(end_pos, end_direction);
    
    % load lut
    lut = LUT(lut_file);
    % define h function
    heuristic_func = @(g, s_p, s_d, e_p, e_d)(hlut(lut, g, s_p, s_d, e_p, e_d));
%     move_validate = @(g, s_p, s_d, e_p, e_d)(check_move(g, s_p, s_d, e_p, e_d, loc2pos_scale));
    move_validate = @(g, s_p, s_d, e_p, e_d)(check_move_polygon(g, s_p, s_d, e_p, e_d, loc2pos_scale));

    tStart = cputime;
    % start Astar search
    [g_score, path] = Astar(graph, start_pos, start_direction, end_pos, end_direction, heuristic_func, move_validate);
%     fprintf("Astar time consumed: %d s\n", cputime-tStart);
    
    % rebuild path
    current_index = end_index;
    if path(current_index+1) == -1
        x = 0;
        y = 0;
        theta = 0;
        path_length = 0;
        is_complete = 0;
        return;
    end
    temp_result_list = {};
    while path(current_index+1) ~= -1
        [pos, dir] = graph.index2position(current_index);
        temp_result_list{length(temp_result_list)+1} = {pos, dir};
        current_index = path(current_index + 1);
    end
    [pos, dir] = graph.index2position(start_index);
    temp_result_list{length(temp_result_list)+1} = {pos, dir};

    result_len = length(temp_result_list);

    % interpolation process
    x = [];
    y = [];
    theta = [];
    for i = result_len:-1:2
        temp_result = temp_result_list{i};
        start_pos = temp_result{1};
        start_direction = temp_result{2};

        temp_result = temp_result_list{i-1};
        end_pos = temp_result{1};
        end_direction = temp_result{2};

        [x_temp, y_temp, theta_temp] = action_set.sample_points(start_pos, start_direction, end_pos, end_direction);
        x = [x; x_temp];
        y = [y; y_temp];
        theta = [theta; theta_temp];

    end
    x = x * loc2pos_scale + planning_scale_.xmin;
    y = y * loc2pos_scale + planning_scale_.ymin;
    path_length = g_score(end_index+1) * loc2pos_scale;
    is_complete = 1;
    time_consumed2 = cputime-tStart;
    
    [x, y, theta] = ResamplePathWithEqualDistance(x, y, theta);
end

function direction = theta2direction(theta)
    % theta2direction. Convert theta to direction representation.
    direction = int32(mod(round(theta / pi * 4), 8));
end

function theta = direction2theta(direction)
    % direction2theta. Convert direction to theta.
%     theta = direction * pi / 4;
    theta = double(direction) * pi / 4;
end

function pos = loc2pos(loc, scale)
    %loc2pos Convert real location to relative position.
    global planning_scale_
    xmin = planning_scale_.xmin;
    ymin = planning_scale_.ymin;
    pos = [int32(floor((loc(1)-xmin)/scale)), int32(floor((loc(2)-ymin)/scale))];
end

function loc = pos2loc(pos, scale)
    %pos2loc convert position to real location
    global planning_scale_
    xmin = planning_scale_.xmin;
    ymin = planning_scale_.ymin;
    loc = [pos(1)*scale+xmin, pos(2)*scale+ymin];
end

function [g_score, path] = Astar(graph, start_pos, start_direction, end_pos, end_direction, heuristic_func, pos_validate)
    %AStar the A*search method.
    %   start_pos: the start position.
    %   start_direction: the start direction.
    %   end_pos: the destination position.
    %   end_direction: the direction when reach end_pos.
    %   heuristic_func: the heuristic fucntion in A* algorithm
    %   pos_validate: judge whether an action is valid.
    path = zeros(1, graph.node_num) - 1;
    nodes = graph.nodes;
    edges = graph.edges;

    start_index = graph.position2index(start_pos, start_direction);
    end_index = graph.position2index(end_pos, end_direction);
    close_index = [];
    open_index = [];
    open_f_score = [];

    % initialize
    g_score = zeros(1, graph.node_num) + inf;
    g_score(start_index + 1) = 0.0;
    h_score = heuristic_func(graph, start_pos, start_direction, end_pos, end_direction);
    f_score = g_score(start_index + 1) + h_score;
    open_index = [open_index, start_index];
    open_f_score = [open_f_score, f_score];

    % start A* iteration
    while ~isempty(open_index)
        [~, f_index] = min(open_f_score);
        open_f_score(f_index) = [];
        current_index = open_index(f_index);
        open_index(f_index) = [];
        close_index = [close_index, current_index];

        [current_pos, current_direction] = graph.index2position(current_index);
        if current_index == end_index
            break;
        end
        current_edges = edges{current_index+1};
        current_edge_num = length(current_edges);
        for i = 1:current_edge_num
            edge = current_edges{i};
            adj_index = edge{1};
            edge_cost= edge{2};

            if ~isempty(close_index(close_index == adj_index))
                continue;
            end

            [adj_pos, adj_direction] = graph.index2position(adj_index);
            if ~ pos_validate(graph, current_pos, current_direction, adj_pos, adj_direction)
                continue;
            end
            temp_g_score = g_score(current_index + 1) + edge_cost;
            if temp_g_score < g_score(adj_index + 1)
                path(adj_index + 1) = current_index;
                g_score(adj_index + 1) = temp_g_score;
                f_score = temp_g_score + heuristic_func(graph, adj_pos, adj_direction, end_pos, end_direction);
                open_index = [open_index, adj_index];
                open_f_score = [open_f_score, f_score];
            end
        end
    end
end

function is_valid = check_move(graph, from_pos, from_direction, to_pos, to_direction, scale)
    %check_move Check whether an action is valid (donot consider the vehicle shape)
    from_loc = pos2loc(from_pos, scale);
    to_loc = pos2loc(to_pos, scale);

    global obstacles_
    obs_len = length(obstacles_);
    for i = 1:obs_len
        obs = obstacles_{i};
        if checkObj_point(to_loc, [obs.x; obs.y])
            is_valid = false;
            return;
        end

        if checkObj_linev(from_loc, to_loc, [obs.x; obs.y])
            is_valid = false;
            return;
        end
    end
    is_valid = true;

end

function is_valid = check_move_polygon(graph, from_pos, from_direction, to_pos, to_direction, scale)
    %check_move Check whether an action is valid (consider the vehicle shape)
    from_loc = pos2loc(from_pos, scale);
    to_loc = pos2loc(to_pos, scale);

    scale_factor = 1.1;

    global obstacles_
    obs_len = length(obstacles_);
    for i = 1:obs_len
        obs = obstacles_{i};
        start_poly = CreateVehiclePolygon(from_loc(1), from_loc(2), direction2theta(from_direction));
        for j = 1:length(start_poly)-1
            point_a = (start_poly(j) - from_loc) * scale_factor + from_loc;
            point_b = (start_poly(j+1) - from_loc) * scale_factor + from_loc;
            if checkObj_linev(point_a, point_b, obs)
                is_valid = false;
                return 
            end
        end

        end_poly = CreateVehiclePolygon(to_loc(1), to_loc(2), direction2theta(to_direction));
        for j = 1:length(start_poly)-1
            point_a = (end_poly(j) - to_loc) * scale_factor + to_loc;
            point_b = (end_poly(j+1) - to_loc) * scale_factor + to_loc;
            if checkObj_linev(point_a, point_b, obs)
                is_valid = false;
                return 
            end
        end
    end
    is_valid = true;

end


function cost = hlut(lut, graph, start_pos, start_direction, end_pos, end_direction)
    %hlut. The HLUT heuristic function. Return the esitimate cost from
    %start_position/direction to end_position/direction.

    edge_length = lut.edge_length;
    delta_pos = end_pos - start_pos;
    
    overflow_factor = 1;
    delta_x = delta_pos(1);
    delta_y = delta_pos(2);
    
    % if delta_x < 0, reflect with y_axis
    if delta_x < 0
        delta_x = - delta_x;
        start_direction = mod(((4 - start_direction) + 8) , 8);
        end_direction = mod(((4 - end_direction) + 8) , 8);
    end

    % if delta_y < 0, reflect with x_axis
    if delta_y < 0
        delta_y = - delta_y;
        start_direction = mod((( - start_direction) + 8) , 8);
        end_direction = mod((( - end_direction) + 8) , 8);
    end

    if (delta_x < edge_length) && (delta_y < edge_length)
        % if the delta pos inside the lut, then directly return the cost.
        cost = lut.look_up(start_direction, [delta_x, delta_y], end_direction);
        return;
    end
    % else use euclidean distance
    cost = double(delta_x*delta_x + delta_y*delta_y)^0.5;

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

function result = checkObj_linev(x1,x2,obj)
    x1 = double(x1);
    x2 = double(x2);
    % Determine whether the line segment formed by x1 and x2 intersects the obstacle
    % The obstacle here is mainly a polygon with four vertices
    result = 1;
    % Number of obstacles
    [n,~] = size(obj);
    nobj = n/2;
    for i = 1:nobj
        index = (i-1) * 2 + 1:i * 2;
        temp_new_obj = obj(index,:);
        % First determine whether the vertex is inside the obstacle
        result1 = checkObj_point(x1,temp_new_obj);
        result2 = checkObj_point(x2,temp_new_obj);
        if result1 == 0 && result2 == 0 % If none of the line segment endpoints are inside the obstacle
            result = 0;
        else
            result = 1;
            break;
        end
        % If the vertices are both outside the obstacle, determine whether the two diagonals intersect the line segment
        % Direction of the line segment
        v1 = x2 - x1;
        % Diagonal1
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

         % Diagonal2
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
    % Determine if xr is inside the obstacle
    % The obstacle here is mainly a polygon with four vertices
    result = 0;
    ncorner = 4;
    xr = double(xr);
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