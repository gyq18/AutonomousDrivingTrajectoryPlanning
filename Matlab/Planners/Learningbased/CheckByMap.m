% Collision detection by checking for dialted map
function Is_collision = CheckByMap(x, y, theta)
    Is_collision = 0;
    global costmap_
    global planning_scale_ hybrid_astar_ vehicle_geometrics_
    if isempty(costmap_) || sum(costmap_,'all') ==0
        costmap_ = CreateDilatedCostmap();
    end
    % The center of the front circle of the vehicle is p1, the center of the rear circle is p2, and the radius of the circumscribed circle is r
    cos_theta = cos(theta);
    sin_theta = sin(theta);

    % Get vehicle Edge Data
    p1x = x + (vehicle_geometrics_.vehicle_length * 3/4 - vehicle_geometrics_.vehicle_rear_hang) * cos_theta;
    p1y = y + (vehicle_geometrics_.vehicle_length * 3/4 - vehicle_geometrics_.vehicle_rear_hang) * sin_theta;

    p2x = x + (vehicle_geometrics_.vehicle_length / 4 - vehicle_geometrics_.vehicle_rear_hang) * cos_theta;
    p2y = y + (vehicle_geometrics_.vehicle_length / 4 - vehicle_geometrics_.vehicle_rear_hang) * sin_theta;

    px = [p1x p2x];
    py = [p1y p2y];
        
    indx = round((px - planning_scale_.xmin) /  hybrid_astar_.resolution_x + 1);
    indy = round((py - planning_scale_.ymin) /  hybrid_astar_.resolution_y + 1);

    if(indx(1) > hybrid_astar_.num_nodes_x || indx(2) > hybrid_astar_.num_nodes_x || indy(2) > hybrid_astar_.num_nodes_y || indy(2) > hybrid_astar_.num_nodes_y)
        Is_collision = 1;
    elseif (sum(costmap_(sub2ind(size(costmap_),indx,indy))))
       Is_collision = 1;
    end
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
for x = 1:hybrid_astar_.num_nodes_x
    costmap(x,1) = 1;
    costmap(x,hybrid_astar_.num_nodes_y) = 1;
end

for y = 1:hybrid_astar_.num_nodes_y
    costmap(1, y) = 1;
    costmap(hybrid_astar_.num_nodes_x, y) = 1;
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