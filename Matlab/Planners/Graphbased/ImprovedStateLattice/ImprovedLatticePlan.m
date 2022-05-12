function [x, y, theta, path_length, is_complete, time_consumed2] = ImprovedLatticePlan(loc2pos_scale, lut_file, interpolation_file)
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
    move_validate = @(g, s_p, s_d, e_p, e_d)(check_move(g, s_p, s_d, e_p, e_d, loc2pos_scale));

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
    


end

function direction = theta2direction(theta)
    % theta2direction. Convert theta to direction representation.
    direction = int32(mod(round(theta / pi * 4), 8));
end

function theta = direction2theta(direction)
    % direction2theta. Convert direction to theta.
    theta = direction * pi / 4;
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
    %check_move Check whether an action is valid.
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