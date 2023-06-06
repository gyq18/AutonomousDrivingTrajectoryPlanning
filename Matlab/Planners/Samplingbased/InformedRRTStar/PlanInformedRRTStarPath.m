function [x, y, theta, path_length, completnessflag] = PlanInformedRRTStarPath()
    global vehicle_TPBV_  num_nodes_s
    rrt = InformedRRTStar([vehicle_TPBV_.x0, vehicle_TPBV_.y0], [vehicle_TPBV_.xtf, vehicle_TPBV_.ytf]);
    [path, path_length] = rrt.informed_rrt_star_search();
    if isempty(path)
        completnessflag = 0;
        x=[];y=[];theta=[];path_length=[];
    else 
        completnessflag = 1;
        x = path(:, 1);
        y = path(:, 2);
        theta = zeros(length(x), 1);
        theta(1) = vehicle_TPBV_.theta0;
        for i = 2:length(theta)
            theta(i) = atan2(y(i) - y(i - 1), x(i) - x(i - 1));
        end
        [x, y, theta] = ResamplePathWithEqualDistance(x, y, theta, num_nodes_s);
    end
end

% resample path with equal distance
function [x, y, theta] = ResamplePathWithEqualDistance(x, y, theta, num_nodes_s)
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
%     temp = temp(1,1:(LARGE_NUM - 1));
    x_extended = [x_extended, temp];
    
    temp = linspace(y(ii), y(ii+1), LARGE_NUM);
%     temp = temp(1,1:(LARGE_NUM - 1));
    y_extended = [y_extended, temp];
    
    temp = linspace(theta(ii), theta(ii+1), LARGE_NUM);
%     temp = temp(1,1:(LARGE_NUM - 1));
    theta_extended = [theta_extended, temp];
end
x_extended = [x_extended, x(end)];
y_extended = [y_extended, y(end)];
theta_extended = [theta_extended, theta(end)];
index = round(linspace(1, length(x_extended), num_nodes_s));
x = x_extended(index);
y = y_extended(index);
theta = theta_extended(index);
end
