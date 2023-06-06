function [x, y, theta, path_length, completeness_flag] = PlanCFSPath()
    global vehicle_TPBV_  num_nodes_s Nobs obstacles_ margin_obs_
    max_iter = 50;
    %% cost function and constraints
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
    
    options = cplexoptimset;
    options.Display = 'off';

    %% initial guess
    [x, y, theta, path_length, completeness_flag] = PlanAStarPath();
    
    num_collision_max = num_nodes_s* Nobs;
    obj=[];
    for i=1:Nobs
        obj=[obj;obstacles_{i}.x;obstacles_{i}.y];
    end
    %% optimization
    if completeness_flag 
        path_initial = zeros(num_nodes_s*dim,1);
        path_initial(1:2:end) = x;
        path_initial(2:2:end) = y;
       
        path_k = path_initial;
        for k = 1:max_iter 
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

            [pathnew,fval,~,~,~] = ...
            cplexqp(Qe,(-Qref*oripath)',A,b,Aeq(:,1:size(Qe,1)),beq,[],[],path_k,options);
%             [pathnew,fval,~,output] = fmincon(@(x) fun(x,Qe,Qref,oripath),path_k,A,b,Aeq(:,1:size(Qe,1)),beq,[],[],[],options);    
            if isempty(pathnew)
                continue;  
            end
            diff = norm(path_k-pathnew);
            if diff < 0.001*num_nodes_s*(dim)
                break
            end
            path_k = pathnew;
        end
        x = pathnew(1:2:end); y = pathnew(2:2:end);
        theta(1) = vehicle_TPBV_.theta0;
        path_length = 0;
        for i = 2:length(theta)
            theta(i) = atan2(y(i) - y(i - 1), x(i) - x(i - 1));
            path_length = path_length + hypot(y(i) - y(i - 1), x(i) - x(i - 1));
        end
    end
end

function [x, y, theta, path_length, completeness_flag] = PlanAStarPath()
    global costmap_
    costmap_ = CreateDilatedCostmap();
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
    expansion_pattern = [-1 1; -1 0; -1 -1; 0 1; 0 -1; 1 1; 1 0; 1 -1];

    Unit = hybrid_astar_.simulation_step/2;

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
        if(completeness_flag == 1)
            break;
        end
    end

    % Output A* path
    cur_best_parent_ind = grid_space_{best_ever_ind(1), best_ever_ind(2), best_ever_ind(3)}(12:14);
    x = grid_space_{best_ever_ind(1), best_ever_ind(2), best_ever_ind(3)}(1);
    y = grid_space_{best_ever_ind(1), best_ever_ind(2), best_ever_ind(3)}(2);
    theta = grid_space_{best_ever_ind(1), best_ever_ind(2), best_ever_ind(3)}(3);

    while (cur_best_parent_ind(1) > -1)
        path_length = path_length + hybrid_astar_.simulation_step/2;
        cur_node = grid_space_{cur_best_parent_ind(1), cur_best_parent_ind(2), cur_best_parent_ind(3)};
        cur_best_parent_ind = cur_node(12:14);
        x = [cur_node(1), x];
        y = [cur_node(2), y];
        theta = [cur_node(3), theta];
        if(x(1)~=x(2)&&y(1)~=y(2))
            path_length = path_length + hybrid_astar_.simulation_step/2*sqrt(2);
        else
            path_length = path_length + hybrid_astar_.simulation_step/2;
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

% Calculation h
function distance_nonholonomic_without_collision_avoidance = CalculateH(start_config, end_config)
    % There is no incomplete distance to avoid collision
    distance_nonholonomic_without_collision_avoidance = norm(start_config(1:2) - end_config(1:2));
end

% Simulate displacement per unit time
function child_node_config = SimulateForUnitDistance(cur_config, Unit_displacement, simulation_step)
    x = cur_config(1);
    y = cur_config(2);
    Unit = simulation_step/2;
    x = Unit * Unit_displacement(1) + x;
    y = Unit * Unit_displacement(2) + y;
    theta = 0;
    child_node_config = [x, y, theta];
end

% (x, y, theta) nodes are valid without collision
function is_collision_free = Is3DNodeValid(child_node_config)
    is_collision_free = 0;
    global vehicle_geometrics_ planning_scale_ hybrid_astar_ costmap_
    xr = child_node_config(:, 1) + vehicle_geometrics_.r2x * cos(child_node_config(:, 3));
    yr = child_node_config(:, 2) + vehicle_geometrics_.r2x * sin(child_node_config(:, 3));
    xf = child_node_config(:, 1) + vehicle_geometrics_.f2x * cos(child_node_config(:, 3));
    yf = child_node_config(:, 2) + vehicle_geometrics_.f2x * sin(child_node_config(:, 3));
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

    indx = round((xx - planning_scale_.xmin) / hybrid_astar_.resolution_x + 1);
    indy = round((yy - planning_scale_.ymin) / hybrid_astar_.resolution_y + 1);

    if (sum(costmap_(sub2ind(size(costmap_), indx, indy))))
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

% Create a scene map with obstacles
function costmap = CreateDilatedCostmap()
    global planning_scale_ hybrid_astar_ vehicle_geometrics_ Nobs obstacles_
    xmin = planning_scale_.xmin;
    ymin = planning_scale_.ymin;
    resolution_x = hybrid_astar_.resolution_x;
    resolution_y = hybrid_astar_.resolution_y;
    costmap = zeros(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y);

    for ii = 1:Nobs
        vx = obstacles_{ii}.x;
        vy = obstacles_{ii}.y;
        x_lb = min(vx); x_ub = max(vx); y_lb = min(vy); y_ub = max(vy);
        [Nmin_x, Nmin_y] = ConvertXYToIndex(x_lb, y_lb);
        [Nmax_x, Nmax_y] = ConvertXYToIndex(x_ub, y_ub);

        for jj = Nmin_x:Nmax_x

            for kk = Nmin_y:Nmax_y

                if (costmap(jj, kk) == 1)
                    continue;
                end

                cur_x = xmin + (jj - 1) * resolution_x;
                cur_y = ymin + (kk - 1) * resolution_y;

                if (inpolygon(cur_x, cur_y, obstacles_{ii}.x, obstacles_{ii}.y) == 1)
                    costmap(jj, kk) = 1;
                end

            end

        end

    end

    length_unit = 0.5 * (resolution_x + resolution_y);
    basic_elem = strel('disk', ceil(vehicle_geometrics_.radius / length_unit));
    costmap = imdilate(costmap, basic_elem);
end

%% Find CFS for all obstacles
function [A, b, d] = FindCFS(xr, obstacle)
    % the point xr:2*1, obstacle 2n*4
    ncorner = size(obstacle, 2); %the number of vertices in the obstacles (4)
    nobj = size(obstacle, 1) / 2;
    d = inf * ones(nobj, 1); 
    % (aTx<=b):pre_A,pre_b
    pre_A = zeros(nobj, 2);
    pre_b = zeros(nobj, 1);
    counter = 0;
    index_true = ones(nobj, 1); % index_true(j)=0 means the jth line is excluded
    % After determining the function phi, F(xr) is: phi(xr) + delta_phi(xr)*(x-xr)>=0
    for j = 1:nobj
        counter = counter + 1;
        obj = obstacle(2 * j - 1:2 * j, :);

        for i = 1:ncorner
            corner1 = obj(:, i);
            corner2 = obj(:, mod(i, ncorner) + 1);
             % xr,corner1 and corner2 form a triangle and find the lengths of the three sides
            dist_r1 = norm(xr - corner1);
            dist_r2 = norm(xr - corner2);
            dist_12 = norm(corner1 - corner2);
            % If angle r12 is obtuse, then xr is closer to corner1 at this point
            if (dist_r1^2 + dist_12^2 - dist_r2^2) < -1e-4 % Cosine Theorem
                temp_d = dist_r1;
                temp_A = xr' - corner1';
                temp_b = temp_A * corner1;
            elseif (dist_r2^2 + dist_12^2 - dist_r1^2) < -1e-4 % If angle r21 is obtuse, then xr is closer to corner2 at this point
                temp_d = dist_r2;
                temp_A = xr' - corner2';
                temp_b = temp_A * corner2;
            else
                % If angle r12 and angle r21 are both acute, then the vertical line from xr to the segment of the line formed by corner1 and corner2 is the shortest
                project_length = (xr - corner1)' * (corner2 - corner1) / dist_12;
                temp_d = sqrt(dist_r1^2 - project_length^2);
                temp_A = [corner1(2) - corner2(2), corner2(1) - corner1(1)];
                temp_b = corner2(1) * corner1(2) - corner1(1) * corner2(2);
            end

            if temp_d < d(j)
                d(j) = temp_d;
                single_A = temp_A;
                single_b = temp_b;
            end

        end

        length_A = norm(single_A);
        single_A = single_A / length_A;
        single_b = single_b / length_A;

        % the diagonal point of the vertex nearest to xr, which should be on either side of the line Ax = b with xr
        for kkk = 1:ncorner

            if single_A * obj(:, kkk) < single_b
                single_A = -single_A;
                single_b = -single_b;
                break;
            end

        end

        pre_A(counter, :) = single_A;
        pre_b(counter, :) = single_b;

    end

    % See if there are redundant constraints in pre_A and pre_b
    % Above, each obstacle corresponds to finding a straight line dividing xr and the obstacle itself. 
    % If excluding this line leaves a line that still splits this obstacle and xr, then this line can indeed be excluded from the constraint
    for j = 1:nobj
        temp_index = index_true;
        temp_index(j) = 0;
        cur_index = find(temp_index > 0);
        cur_A = pre_A(cur_index, :);
        cur_b = pre_b(cur_index);
        obj = obstacle(2 * j - 1:2 * j, :);

        result = cur_A * obj - repmat(cur_b, 1, 4);
        flag = sum(result >= 0, 2);

        if ~isempty(find(flag >= 4, 1))
            index_true(j) = 0;
        end

    end

    final_index = find(index_true > 0);
    A = pre_A(final_index, :);
    b = pre_b(final_index);
end