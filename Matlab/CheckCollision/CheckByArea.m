% Collision detection by comparative area method
% Algorithm reference https://zhuanlan.zhihu.com/p/449795053
function Is_collision = CheckByArea(x, y, theta)
    global obstacles_ Nobs
    % Get vehicle Edge Data
    V = CreateVehiclePolygon(x, y, theta);
    Is_collision = 0;
    for j = 1:Nobs
        vertex_x = obstacles_{j}.x;
        vertex_y = obstacles_{j}.y;
        obj=[vertex_x;vertex_y]; 
        for i = 1:4
            Collision = P_Comparative_Area_Collision(V(:, i), obj);
            if (Collision == 1)
                Is_collision = 1;
            end
        end
        for i = 1:4
            Collision = P_Comparative_Area_Collision(obj(:, i), V);
            if (Collision == 1)
                Is_collision = 1;
            end
        end
        if Is_collision
            break;
        end
    end
end

% Collision detection of point p and obstacle V by comparative area method
function Is_collision = P_Comparative_Area_Collision(p, obj)
    ncorner = 4;
    obj_area = triArea(obj(:, 1),obj(:, 2),obj(:, 3)) + triArea(obj(:, 1),obj(:, 4),obj(:, 3));
    p_area = 0;
    for i = 1:ncorner
        p_area = p_area + triArea(p, obj(:, mod(i, ncorner) + 1), obj(:, mod(i + 1, ncorner) + 1));
    end
    if (p_area > obj_area + 0.1)
        Is_collision = 0;
    else
        Is_collision = 1;
    end
end

% Triangle area
function area = triArea(p1, p2, p3)
    a = norm(p1 - p2);
    b = norm(p1 - p3);
    c = norm(p2 - p3);
    half = (a + b + c) / 2;
    area = sqrt(half * (half - a) * (half - b) * (half - c));
end

%% calculate of the vehicle edge position based on the position of the rear axle center
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
