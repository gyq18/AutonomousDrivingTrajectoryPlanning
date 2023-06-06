function [x, y, theta, path_length, completeness_flag] = PlanAStarPath()
    global hybrid_astar_ vehicle_TPBV_
    grid_space_ = cell(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y, hybrid_astar_.num_nodes_theta);
    end_config = [vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf];
    start_config = [vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0];
    goal_ind = Convert3DimConfigToIndex(end_config);

    init_node = zeros(1, 14);
    % Information of each element in each node:
    % Dim # | Variable
    %  1        x
    %  2        y
    %  3        theta
    %  4        f
    %  5        g
    %  6        h
    %  7        is_in_openlist
    %  8        is_in_closedlist
    %  9-11     index of current node
    %  12-14    index of parent node

    init_node(1:3) = start_config;
    init_node(6) = CalculateH(start_config, end_config);
    init_node(5) = 0;
    init_node(4) = init_node(5) + hybrid_astar_.multiplier_H * init_node(6);
    init_node(7) = 1;
    init_node(9:11) = Convert3DimConfigToIndex(start_config);
    init_node(12:14) = [-999, -999, -999];
    openlist_ = init_node;

    grid_space_{init_node(9), init_node(10), init_node(11)} = init_node;
    % 8 child nodes
    expansion_pattern = [-1 1 3 * pi / 4; -1 0 -pi; -1 -1 -3 * pi / 4; 0 1 pi / 2; 0 -1 -pi / 2; 1 1 pi / 4; 1 0 0; 1 -1 -pi / 4];

    Unit = hybrid_astar_.simulation_step / 2;

    expansion_length = Unit * [1.414; 1; 1.414; 1; 1; 1.414; 1; 1.414];
    completeness_flag = 0;
    best_ever_val = Inf;
    best_ever_ind = init_node(9:11);
    path_length = 0;
    iter = 0;

    tic

    % Enter and move out of the list until the number of steps or time runs out or arrives
    while ((~isempty(openlist_)) && (iter <= hybrid_astar_.max_iter) && (~completeness_flag) && (toc <= hybrid_astar_.max_time))
        iter = iter + 1;
        cur_node_order = find(openlist_(:, 4) == min(openlist_(:, 4)));
        cur_node_order = cur_node_order(end);
        cur_node = openlist_(cur_node_order, :);
        cur_config = cur_node(1:3);
        cur_ind = cur_node(9:11);
        cur_g = cur_node(5);

        % Remove cur_node from openlist and add it in closed list
        openlist_(cur_node_order, :) = [];
        grid_space_{cur_ind(1), cur_ind(2), cur_ind(3)}(7) = 0;
        grid_space_{cur_ind(1), cur_ind(2), cur_ind(3)}(8) = 1;
        % Expand the current node to its 8 children
        for ii = 1:8

            child_node_config = SimulateForUnitDistance(cur_config, expansion_pattern(ii, :), hybrid_astar_.simulation_step);
            child_node_ind = Convert3DimConfigToIndex(child_node_config);
            % If the child node has been explored ever before, and it has been closed:
            if ((~isempty(grid_space_{child_node_ind(1), child_node_ind(2), child_node_ind(3)})) && (grid_space_{child_node_ind(1), child_node_ind(2), child_node_ind(3)}(8) == 1))
                continue;
            end

            child_g = cur_g + expansion_length(ii);

            % Now, if the child node has been explored ever before, and it is still in the openlist:
            if (~isempty(grid_space_{child_node_ind(1), child_node_ind(2), child_node_ind(3)}))
                % If the previously found parent of the child is not good enough, a change is to be made
                if (grid_space_{child_node_ind(1), child_node_ind(2), child_node_ind(3)}(5) > child_g + 0.1)
                    child_node_order1 = find(openlist_(:, 9) == child_node_ind(1));
                    child_node_order2 = find(openlist_(child_node_order1, 10) == child_node_ind(2));
                    child_node_order3 = find(openlist_(child_node_order1(child_node_order2), 11) == child_node_ind(3));
                    openlist_(child_node_order1(child_node_order2(child_node_order3)), :) = [];
                    child_node_update = grid_space_{child_node_ind(1), child_node_ind(2), child_node_ind(3)};
                    child_node_update(5) = child_g;
                    child_node_update(4) = child_node_update(5) + hybrid_astar_.multiplier_H * child_node_update(6);
                    child_node_update(12:14) = cur_ind;
                    openlist_ = [openlist_; child_node_update];
                    grid_space_{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node_update;

                end

                continue;
            end

            % Now the child node is ensured to be newly expanded
            if (~Is3DNodeValid(child_node_config))
                child_node(8) = 1;
                grid_space_{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node;
                continue;
            end

            % We did not calculate H for the child node till now to avoid wasiting CPU.
            % Now the child node is both new and collision-free.
            child_node(1:3) = child_node_config;
            child_node(5) = child_g;
            child_node(6) = CalculateH(child_node_config, end_config);
            child_node(4) = child_node(5) + hybrid_astar_.multiplier_H * child_node(6);
            child_node(7) = 1;
            child_node(9:11) = child_node_ind;
            child_node(12:14) = cur_ind;
            openlist_ = [openlist_; child_node];
            grid_space_{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node;
            % Failure-safe solution preparation
            if (child_node(6) < best_ever_val)
                best_ever_val = child_node(6);
                best_ever_ind = child_node_ind;
            end

            % If child node is the goal node
            if child_node_ind(1) == goal_ind(1) && child_node_ind(2) == goal_ind(2)
                completeness_flag = 1;
                best_ever_ind = goal_ind;
                break;
            end

        end

        if (completeness_flag == 1)
            break;
        end

    end

    % Output A* path
    cur_best_parent_ind = grid_space_{best_ever_ind(1), best_ever_ind(2), best_ever_ind(3)}(12:14);
    x = grid_space_{best_ever_ind(1), best_ever_ind(2), best_ever_ind(3)}(1);
    y = grid_space_{best_ever_ind(1), best_ever_ind(2), best_ever_ind(3)}(2);
    theta = grid_space_{best_ever_ind(1), best_ever_ind(2), best_ever_ind(3)}(3);

    while (cur_best_parent_ind(1) > -1)
        path_length = path_length + hybrid_astar_.simulation_step / 2;
        cur_node = grid_space_{cur_best_parent_ind(1), cur_best_parent_ind(2), cur_best_parent_ind(3)};
        cur_best_parent_ind = cur_node(12:14);
        x = [cur_node(1), x];
        y = [cur_node(2), y];
        theta = [cur_node(3), theta];

        if (x(1) ~= x(2) && y(1) ~= y(2))
            path_length = path_length + hybrid_astar_.simulation_step / 2 * sqrt(2);
        else
            path_length = path_length + hybrid_astar_.simulation_step / 2;
        end

    end

    if (completeness_flag)
        x = [x, end_config(1)];
        y = [y, end_config(2)];
        theta = [theta, end_config(3)];

        path_length = 0;

        for ii = 1:(length(x) - 1)
            path_length = path_length + hypot(x(ii + 1) - x(ii), y(ii + 1) - y(ii));
        end

    end

    if (completeness_flag)
        [x, y, theta] = ResamplePathWithEqualDistance(x, y, theta);
    end

end

function [x, y, theta] = ResamplePathWithEqualDistance(x, y, theta)

    for ii = 2:length(theta)

        while (theta(ii) - theta(ii - 1) > pi)
            theta(ii) = theta(ii) - 2 * pi;
        end

        while (theta(ii) - theta(ii - 1) < -pi)
            theta(ii) = theta(ii) + 2 * pi;
        end

    end

    x_extended = [];
    y_extended = [];
    theta_extended = [];

    for ii = 1:(length(x) - 1)
        distance = hypot(x(ii + 1) - x(ii), y(ii + 1) - y(ii));
        LARGE_NUM = round(distance * 100);
        temp = linspace(x(ii), x(ii + 1), LARGE_NUM);
        temp = temp(1, 1:(LARGE_NUM - 1));
        x_extended = [x_extended, temp];

        temp = linspace(y(ii), y(ii + 1), LARGE_NUM);
        temp = temp(1, 1:(LARGE_NUM - 1));
        y_extended = [y_extended, temp];

        temp = linspace(theta(ii), theta(ii + 1), LARGE_NUM);
        temp = temp(1, 1:(LARGE_NUM - 1));
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

% Calculation h
function distance_nonholonomic_without_collision_avoidance = CalculateH(start_config, end_config)
    % There is no incomplete distance to avoid collision
    distance_nonholonomic_without_collision_avoidance = norm(start_config(1:2) - end_config(1:2));
end

% Simulate displacement per unit time
function child_node_config = SimulateForUnitDistance(cur_config, Unit_displacement, simulation_step)
    x = cur_config(1);
    y = cur_config(2);
    Unit = simulation_step / 2;
    x = Unit * Unit_displacement(1) + x;
    y = Unit * Unit_displacement(2) + y;
    theta = Unit_displacement(3);
    child_node_config = [x, y, theta];
end

% (x, y, theta) nodes are valid without collision
function is_collision_free = Is3DNodeValid(child_node_config)
    is_collision_free = 0;

    if (CheckByMap(child_node_config(:, 1), child_node_config(:, 2), child_node_config(:, 3)))
        return;
    end

    is_collision_free = 1;
end

% (x, y) rounding up standardization
function [ind1, ind2] = ConvertXYToIndex(x, y)
    global hybrid_astar_ planning_scale_
    ind1 = ceil((x - planning_scale_.xmin) / hybrid_astar_.resolution_x) + 1;
    ind2 = ceil((y - planning_scale_.ymin) / hybrid_astar_.resolution_y) + 1;

    if ((ind1 <= hybrid_astar_.num_nodes_x) && (ind1 >= 1) && (ind2 <= hybrid_astar_.num_nodes_y) && (ind2 >= 1))
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

% Normalize (x, y, theta) by rounding it up
function idx = Convert3DimConfigToIndex(config)
    global hybrid_astar_ planning_scale_
    ind1 = ceil((config(1) - planning_scale_.xmin) / hybrid_astar_.resolution_x) + 1;
    ind2 = ceil((config(2) - planning_scale_.ymin) / hybrid_astar_.resolution_y) + 1;
    ind3 = 1;
    idx = [ind1, ind2, ind3];

    if ((ind1 <= hybrid_astar_.num_nodes_x) && (ind1 >= 1) && (ind2 <= hybrid_astar_.num_nodes_y) && (ind2 >= 1))
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

    idx = [ind1, ind2, ind3];
end
