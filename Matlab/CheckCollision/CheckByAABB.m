%  AABB collision detection
function Is_collision = CheckByAABB(x, y, theta)
    global obstacles_ Nobs
    % Get vehicle Edge Data
    V = CreateVehiclePolygon(x, y, theta);
    % Set vehicle projection data
    A = Min(V);
    B = Max(V);
    V1 = [A B]; %V1 = [A; B];
    % Set obj projection data
    for i = 1:Nobs
        vertex_x = obstacles_{i}.x;
        vertex_y = obstacles_{i}.y;
        obj=[vertex_x;vertex_y];
        C = Min(obj);
        D = Max(obj);
        V2 = [C D]; %V2 = [C; D];
        % Judge whether there is collision between projections
        Is_collision = AABB_Quadrilateral_Collision(V1, V2);
        if Is_collision
            break;
        end
    end
end

% AABB projection quadrilateral collision detection
function Is_collision = AABB_Quadrilateral_Collision(V1, V2)
    IsCollisionx = 0;
    IsCollisiony = 0;

    for i = 1:2
        % X-axis
        if (V1(1, i) - V2(1, 1)) * (V1(1, i) - V2(1, 2)) <= 0
            IsCollisionx = 1;
        end

        % Y-axis
        if (V1(2, i) - V2(2, 1)) * (V1(2, i) - V2(2, 2)) <= 0
            IsCollisiony = 1;
        end

    end

    for i = 1:2

        if (V2(1, i) - V1(1, 1)) * (V2(1, i) - V1(1, 2)) <= 0
            IsCollisionx = 1;
        end

        if (V2(2, i) - V1(2, 1)) * (V2(2, i) - V1(2, 2)) <= 0
            IsCollisiony = 1;
        end

    end

    if IsCollisionx == 1 && IsCollisiony == 1
        Is_collision = 1;
    else
        Is_collision = 0;
    end

end

% Calculates a set of non adjacent points of a projected quadrilateral
function m = Min(V)
    minx = V(1, 1);
    miny = V(2, 1);
    for i = 2:4
        if V(1, i) < minx
            minx = V(1, i);
        end
        if V(2, i) < miny
            miny = V(2, i);
        end
    end
    m = [minx; miny];
end

function m = Max(V)
    maxx = V(1, 1);
    maxy = V(2, 1);
    for i = 2:4
        if V(1, i) > maxx
            maxx = V(1, i);
        end
        if V(2, i) > maxy
            maxy = V(2, i);
        end
    end
    m = [maxx; maxy];
end

% calculate of the vehicle edge position based on the position of the rear axle center
function V = CreateVehiclePolygon(x, y, theta)
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
%     V.x = [AX, BX, CX, DX, AX];
%     V.y = [AY, BY, CY, DY, AY];
    V = [AX, BX, CX, DX, AX;AY, BY, CY, DY, AY];
end
