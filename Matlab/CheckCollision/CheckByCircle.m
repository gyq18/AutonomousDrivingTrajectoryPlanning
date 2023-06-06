% Whether the front and rear circumscribed circles of the vehicle collide with obstacles
function Is_collision = CheckByCircle(x, y, theta)
    global obstacles_ Nobs
    global vehicle_geometrics_
    ncorner = 4;
    Is_collision = 0;
    % The center of the front circle of the vehicle is p1, the center of the rear circle is p2, and the radius of the circumscribed circle is r
    cos_theta = cos(theta);
    sin_theta = sin(theta);
    r = sqrt((vehicle_geometrics_.vehicle_length / 4)^2 + (vehicle_geometrics_.vehicle_width / 2)^2);

    p1x = x + (vehicle_geometrics_.vehicle_length * 3/4 - vehicle_geometrics_.vehicle_rear_hang) * cos_theta;
    p1y = y + (vehicle_geometrics_.vehicle_length * 3/4 - vehicle_geometrics_.vehicle_rear_hang) * sin_theta;

    p2x = x + (vehicle_geometrics_.vehicle_length / 4 - vehicle_geometrics_.vehicle_rear_hang) * cos_theta;
    p2y = y + (vehicle_geometrics_.vehicle_length / 4 - vehicle_geometrics_.vehicle_rear_hang) * sin_theta;

    p1 = [p1x; p1y];
    p2 = [p2x; p2y];

    for j = 1:Nobs
        vertex_x = obstacles_{j}.x;
        vertex_y = obstacles_{j}.y;
        obj=[vertex_x;vertex_y];
        for i = 1:ncorner
            if i == 4
                Collision = Circumscribed_Circle(p1, r, obj(:, 4), obj(:, 1));
            else
                Collision = Circumscribed_Circle(p1, r, obj(:, i), obj(:,i+1));
            end
            if (Collision == 1)
                Is_collision = 1;
            end
        end
        for i = 1:ncorner
             if i == 4
                Collision = Circumscribed_Circle(p2, r, obj(:, 4), obj(:, 1));
            else
                Collision = Circumscribed_Circle(p2, r, obj(:, i), obj(:,i+1));
             end
            if (Collision == 1)
                Is_collision = 1;
            end
        end
        if Is_collision == 0
            Is_collision = check_in(p1, obj);
        end
        if Is_collision
            break;
        end
    end

end

% Whether the circle with P as the center and r as the radius intersects with the line segment o1o2
function Is_Collision = Circumscribed_Circle(p, r, o1, o2)
    Is_Collision = 0;
    % First, determine whether o1o2 is within the circumscribed circle
    if (norm(p - o1) <= r)
        Is_Collision = 1;
    end

    if (norm(p - o2) <= r)
        Is_Collision = 1;
    end

    % If o1o2 they are all outside the obstacle, determine whether the circle intersects the line segment
    norm_dist = norm(o2 - o1);

    if (norm_dist < 0.000001)
        Is_Collision = 2;
    else
        A =- (o2(2) - o1(2));
        B = o2(1) - o1(1);
        % Distance from circle center to line segment
        dist = abs(A * (p(1) - o1(1)) + B * (p(2) - o1(2))) / sqrt(A^2 + B^2);

        if (dist > r)
            Is_Collision = 2;
        else
            m = B * p(1) - A * p(2);
            % Calculate the coordinates of the intersection of the vertical line of the center of the circle and the straight line o1o2
            x4 = (A^2 * o1(1) + B * (m + o1(2) * A)) / (A^2 + B^2);

            if (abs(B) > 0.000001)

                if ((o1(1) - x4) * (o2(1) - x4) < 0)
                    Is_Collision = 1;
                end

            else
                y4 = (B * x4 - m) / A;

                if ((o1(2) - y4) * (o2(2) - y4) < 0)
                    Is_Collision = 1;
                end

            end

        end

    end

    if (Is_Collision == 2)
        Is_Collision = 0;
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

% Whether the detection point is in the obstacle
function Is_Collision = check_in(p, obj)
    S1 = triArea(p,obj(:, 1),obj(:, 4));
    for i = 1:3
        S1 = S1 + triArea(p,obj(:, i),obj(:, i+1));
    end
    S2 = triArea(obj(:, 1),obj(:, 2),obj(:, 3)) + triArea(obj(:, 1),obj(:, 4),obj(:, 3));
    Is_Collision = 0;
    if(S1 == S2)
        Is_Collision = 1;
    end
end