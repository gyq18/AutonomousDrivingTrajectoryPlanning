% OBB collision detection
% https://blog.csdn.net/silangquan/article/details/50812425
function Is_collision = CheckByOBB(x, y, theta, obj)
    % Get vehicle Edge Data
    V = CreateVehiclePolygon(x, y, theta);
    Is_collision = OBB_Quadrilateral_Collision(V, obj);
end

% OBB projection quadrilateral collision detection
function Is_collision = OBB_Quadrilateral_Collision(V1, V2)
    % coordinate position of the center of rectangle A
    PA = (V1(:, 1) + V1(:, 2) + V1(:, 3) + V1(:, 4)) / 4;
    % half width of A (corresponds with the local x-axis of A)
    WA = norm(V1(:, 4) - V1(:, 1)) / 2;
    % half height of A (corresponds with the local y-axis of A)
    HA = norm(V1(:, 2) - V1(:, 1)) / 2;
    % unit vector representing the local x-axis of A
    Ax = (V1(:, 4) - V1(:, 1)) / (2 * WA);
    % unit vector representing the local y-axis of A
    Ay = (V1(:, 2) - V1(:, 1)) / (2 * HA);

    % coordinate position of the center of rectangle B
    PB = (V2(:, 1) + V2(:, 2) + V2(:, 3) + V2(:, 4)) / 4;
    % half width of A (corresponds with the local x-axis of B)
    WB = norm(V2(:, 4) - V2(:, 1)) / 2;
    % half height of A (corresponds with the local y-axis of B)
    HB = norm(V2(:, 2) - V2(:, 1)) / 2;
    % unit vector representing the local x-axis of B
    Bx = (V2(:, 4) - V2(:, 1)) / (2 * WB);
    % unit vector representing the local y-axis of B
    By = (V2(:, 2) - V2(:, 1)) / (2 * HB);

    T = PB - PA;

    % Separating Axis Judge
    Separating_Axis = 0;

    % CASE 1:
    % L = Ax
    % | T â€? Ax | > WA + | ( WB*Bx ) â€? Ax | + |( HB*By ) â€? Ax |
    % If true, there is a separating axis parallel Ax.
    if (abs(T .* Ax) > WA + abs((WB * Bx) .* Ax) + abs((HB * By) .* Ax))
        Separating_Axis = Separating_Axis + 1;
    end

    % CASE 2:
    % L = Ay
    % | T â€? Ay | > HA + | ( WB*Bx ) â€? Ay | + |( HB*By ) â€? Ay |
    % If true, there is a separating axis parallel Ax.
    if (abs(T .* Ay) > HA + abs((WB * Bx) .* Ay) + abs((HB * By) .* Ay))
        Separating_Axis = Separating_Axis + 1;
    end

    % CASE 3:
    % L = Bx
    % | T â€? Bx | > | ( WA* Ax ) â€? Bx | + | ( HA*Ay ) â€? Bx | + WB
    % If true, there is a separating axis parallel Bx.
    if (abs(T .* Bx) > WB + abs((WA * Ax) .* Bx) + abs((HA * Ay) .* Bx))
        Separating_Axis = Separating_Axis + 1;
    end

    % CASE 4:
    % L = By
    % | T â€? By | > | ( WA* Ax ) â€? By | + | ( HA*Ay ) â€? By | + HB
    % If true, there is a separating axis parallel By.
    if (abs(T .* By) > HB + abs((WA * Ax) .* By) + abs((HA * Ay) .* By))
        Separating_Axis = Separating_Axis + 1;
    end

    if (Separating_Axis > 0)
        Is_collision = 0;
    else
        Is_collision = 1;
    end

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
    V.x = [AX, BX, CX, DX, AX];
    V.y = [AY, BY, CY, DY, AY];
end
