% Whether the front and rear circumscribed circles of the vehicle collide with obstacles
function Is_Collision = CheckByCircle(x, y, theta, obj)
    global vehicle_geometrics_
    ncorner = 4;
    Is_Collision = 0;
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

    for i = 1:ncorner
        Collision = Circumscribed_Circle(p1, r, obj(:, i), obj(:, mod(i + 1, ncorner)));

        if (Collision == 1)
            Is_Collision = 1;
        end

    end

    for i = 1:ncorner
        Collision = Circumscribed_Circle(p2, r, obj(:, i), obj(:, mod(i + 1, ncorner)));

        if (Collision == 1)
            Is_Collision = 1;
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
            x4 = (A^2 * o1(1) - B * (m - o1(2) * A)) / (A^2 + B^2);

            if (abs(B) > 0.000001)

                if ((o1(1) - x4) * (o2(1) - x4) < 0)
                    Is_Collision = 1;
                end

            else
                y4 = (B * x4 + m) / A;

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
