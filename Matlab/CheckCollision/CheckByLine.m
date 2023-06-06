% Collision detection by checking for lines and vertices of the gnerated rectangle (vehicle)
function Is_collision = CheckByLine(x, y, theta)
    Is_collision = 0;
    global obstacles_ Nobs
    % Get vehicle Edge Data
    V = CreateVehiclePolygon(x, y, theta);

    for i = 1:Nobs
        vertex_x = obstacles_{i}.x;
        vertex_y = obstacles_{i}.y;
        obj=[vertex_x;vertex_y];
        flag1 = checkObj_linev(V(:,1),V(:,2),obj);
        flag2 = checkObj_linev(V(:,2),V(:,3),obj);
        flag3 = checkObj_linev(V(:,3),V(:,4),obj);
        flag4 = checkObj_linev(V(:,4),V(:,1),obj);  
        if flag1+flag2+flag3+flag4 > 0
             Is_collision = 1;
        end  
        if Is_collision
            break;
        end
    end
end

% Determine whether the line segment formed by x1 and x2 intersects the obstacle
function result = checkObj_linev(x1,x2,obj)
    x1 = double(x1);
    x2 = double(x2);
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
        v1 = x2 - x1;    % Direction of the line segment
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

% Determine if xr is inside the obstacle
function result = checkObj_point(xr,obj)
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