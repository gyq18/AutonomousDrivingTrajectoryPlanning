function [x, y, theta, path_length, completeness_flag] = PlanKinematicRRTStarPath()
    global costmap_  vehicle_TPBV_
    costmap_ = CreateDilatedCostmap();
    costmap = vehicleCostmap(costmap_);
    startPose = [vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0*180/pi];
    goalPose  = [vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf*180/pi];
    planner = pathPlannerRRT(costmap);
    refPath = plan(planner,startPose,goalPose);
    if checkPathValidity(refPath,costmap)
        transitionPoses = interpolate(refPath);
        x = transitionPoses(:,1);
        y = transitionPoses(:,2);
        theta = transitionPoses(:,3)*pi/180;
        completeness_flag = 1;
    else 
        completeness_flag = 0;
    end    
    if (completeness_flag)
        [x, y, theta] = ResamplePathWithEqualDistance(x, y, theta);
    end
    path_length = 0;
    for ii = 1:(length(x) - 1)
        path_length = path_length + hypot(x(ii + 1) - x(ii), y(ii + 1) - y(ii));
    end
end

function costmap = CreateDilatedCostmap()
    global planning_scale_ hybrid_astar_ vehicle_geometrics_ Nobs obstacles_
    xmin = planning_scale_.xmin;
    ymin = planning_scale_.ymin;
    resolution_x = hybrid_astar_.resolution_x;
    resolution_y = hybrid_astar_.resolution_y;
    hybrid_astar_.grid_num_x = ceil(planning_scale_.x_scale / hybrid_astar_.resolution_x);
    hybrid_astar_.grid_num_y = ceil(planning_scale_.y_scale / hybrid_astar_.resolution_y);
    costmap = zeros(hybrid_astar_.grid_num_x, hybrid_astar_.grid_num_y);

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
    if ((ind1 <= hybrid_astar_.grid_num_x)&&(ind1 >= 1)&&(ind2 <= hybrid_astar_.grid_num_y)&&(ind2 >= 1))
        return;
    end
    if (ind1 > hybrid_astar_.grid_num_x)
        ind1 = hybrid_astar_.grid_num_x;
    elseif (ind1 < 1)
        ind1 = 1;
    end
    if (ind2 > hybrid_astar_.grid_num_y)
        ind2 = hybrid_astar_.grid_num_y;
    elseif (ind2 < 1)
        ind2 = 1;
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