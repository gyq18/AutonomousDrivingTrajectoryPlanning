function trajectory = TransformPathToTrajectory(x, y, theta, path_length, method_flag)
%% method_flag:1-FulfillProfiles(x, y, theta);2-GYQProfiles
    trajectory.x=x; trajectory.y=y; 
    if method_flag==1
        [trajectory.theta, trajectory.v, trajectory.a, trajectory.phi, trajectory.omega] = FulfillProfiles(x, y, theta, path_length);
    elseif method_flag==2
        [trajectory.theta, trajectory.v, trajectory.a, trajectory.phi, trajectory.omega] = GYQProfiles(x, y, theta);
    end
end

function [theta,v, a, phi, omega] = FulfillProfiles(x, y, theta, path_length)
Nfe = length(x);
% Judge velocity direction
vdr = zeros(1, Nfe);
for ii = 2 : (Nfe - 1)
    addtion = (x(ii+1) - x(ii)) * cos(theta(ii)) + (y(ii+1) - y(ii)) * sin(theta(ii));
    if (addtion > 0)
        vdr(ii) = 1;
    else
        vdr(ii) = -1;
    end
end
v = zeros(1, Nfe);
a = zeros(1, Nfe);
dt = path_length / Nfe;
for ii = 2 : Nfe
    v(ii) = vdr(ii) * sqrt(((x(ii) - x(ii-1)) / dt)^2 + ((y(ii) - y(ii-1)) / dt)^2);
end
for ii = 2 : Nfe
    a(ii) = (v(ii) - v(ii-1)) / dt;
end
phi = zeros(1, Nfe);
omega = zeros(1, Nfe);
global vehicle_kinematics_ vehicle_geometrics_
phi_max = vehicle_kinematics_.vehicle_phi_max;
omega_max = vehicle_kinematics_.vehicle_omega_max;
    for ii = 2 : (Nfe-1)
        phi(ii) = atan((theta(ii+1) - theta(ii)) * vehicle_geometrics_.vehicle_wheelbase / (dt * v(ii)));
        if (phi(ii) > phi_max)
            phi(ii) = phi_max;
        elseif (phi(ii) < -phi_max)
            phi(ii) = -phi_max;
        end
    end
    for ii = 2 : (Nfe-1)
        omega(ii) = (phi(ii+1) - phi(ii)) / dt;
        if (omega(ii) > omega_max)
            omega(ii) = omega_max;
        elseif (omega(ii) < -omega_max)
            omega(ii) = -omega_max;
        end
    end
end

function [theta,v, a, phi, omega] = GYQProfiles(x, y, theta)
global vehicle_kinematics_ vehicle_geometrics_
nstep = length(x);
[tlength,max_dlength] = CalculateLength(x,y);

delat_t2 = max_dlength /vehicle_kinematics_.vehicle_v_max; % The second one is based on the maximum distance per unit to find the unit time

theta_initial = theta(1);
theta_end = theta(nstep);
max_dtheta=0;
for i = 2:nstep-1  
    x1 = [x(i),y(i)];
    x2 = [x(i+1),y(i+1)];
    if x2(2)==x1(2) && x2(1)==x1(1)  % discontinuous points of curvature change
         theta(i)=0;
    elseif x2(2)>x1(2) && x2(1)<x1(1) % second quadrant
          theta(i)=atan((x2(2)-x1(2))/(x2(1)-x1(1)))+pi;
%           if path_dir(i)<0  % reverse
%               theta(i)=theta(i)+pi;
%           end
    elseif x2(2)<x1(2) && x2(1)<x1(1) % third quadrant
          theta(i)=atan((x2(2)-x1(2))/(x2(1)-x1(1)))+pi;
%           if path_dir(i)<0  % reverse
%               theta(i)=pi-theta(i);
%           end
    elseif x2(2)<x1(2) && x2(1)>x1(1) % fourth quadrant
          theta(i)=atan((x2(2)-x1(2))/(x2(1)-x1(1)))+2*pi;
%           if path_dir(i)<0  % reverse
%               theta(i)=theta(i)-pi;
%           end
    else
         theta(i) = atan((x2(2)-x1(2))/(x2(1)-x1(1)));  % first quadrant
%           if path_dir(i)<0  % reverse
%               theta(i)=theta(i)+pi;
%           end
    end
    if  theta(i)- theta(i-1)>pi/2  % if the adjacent change is too large
         theta(i)= theta(i)-2*pi;
    end
end
theta(1)=theta_initial;
theta(end)=theta_end;
for i=1:nstep-1
     if max_dtheta<theta(i+1)-theta(i)
        max_dtheta=theta(i+1)-theta(i);
     end
end

% delat_t3 = max_dtheta*Lm /vehicle_kinematics_.vehicle_v_max/tan(vehicle_kinematics_.vehicle_phi_max);  %第三种根据单位最大角度变化量来求单位时间
delat_t3 = 0;

% 计算初始车轮偏角
phi = zeros(nstep,1);
for i=1:(nstep-1)
    dlength=norm([x(i+1),y(i+1)]-[x(i),y(i)]);
%     if path_dir(i)<0  %倒退
%         dlength=-dlength;
%     end
    phi(i)=atan((theta(i+1)-theta(i))* vehicle_geometrics_.vehicle_wheelbase/dlength);
end
phi(nstep) = phi(nstep-1);  
max_phi=0;
for i=1:nstep-1
     if max_phi<phi(i+1)-phi(i)
        max_phi=phi(i+1)-phi(i);
     end
end
delat_t4= max_phi/vehicle_kinematics_.vehicle_omega_max;  %第四种根据单位最大角加速度度变化量来求单位时间

delat_t5= (vehicle_kinematics_.vehicle_v_max-(-vehicle_kinematics_.vehicle_v_max))/vehicle_kinematics_.vehicle_a_max;  %第五种根据单位最大加速度度变化量来求单位时间

delat_t = max(delat_t2,delat_t3);
delat_t = max(delat_t,delat_t4);
delat_t = max(delat_t,delat_t5);

delat_t = tlength /vehicle_kinematics_.vehicle_v_max/(nstep-1);

tf0 = delat_t*(nstep-1);


v = zeros(nstep,1);
for i = 1:nstep-1
        x1 = [x(i),y(i)];
        x2 = [x(i+1),y(i+1)];
        v(i) = norm(x2-x1)/delat_t;
%         if path_dir(i)<0  %倒退
%              v(i)= -v(i);
%         end
end
v(nstep) = v(nstep-1);

interval = delat_t;
%% 将v,phi,a,omega限制在允许的最大阈值之内
delta_v = vehicle_kinematics_.vehicle_a_max*interval;
delta_phi = vehicle_kinematics_.vehicle_omega_max*interval;

for i = 2:nstep
    temp1 = v(i)-v(i-1);
    if abs(temp1)>delta_v
        v(i) = v(i-1)+sign(temp1)*delta_v;
    end
    temp2 = phi(i)-phi(i-1);
    if abs(temp2)>delta_phi
        phi(i) = phi(i-1)+sign(temp2)*delta_phi;
    end
end
v = min(vehicle_kinematics_.vehicle_v_max,v);
v = max(-vehicle_kinematics_.vehicle_v_max,v);
phi = min(vehicle_kinematics_.vehicle_phi_max,phi);
phi = max(-vehicle_kinematics_.vehicle_phi_max,phi);
a = zeros(1, Nfe);
for ii = 2 : Nfe
    a(ii) = (v(ii) - v(ii-1)) / dt;
end
omega = zeros(1, Nfe);
omega_max = vehicle_kinematics_.vehicle_omega_max;
for ii = 2 : (Nfe-1)
    omega(ii) = (phi(ii+1) - phi(ii)) / dt;
    if (omega(ii) > omega_max)
        omega(ii) = omega_max;
    elseif (omega(ii) < -omega_max)
        omega(ii) = -omega_max;
    end
end
end

function [tlength,max_dlength] = CalculateLength(path_x,path_y)
tlength=0;
max_dlength=0; %单位变化内的最大距离
nstep = length(path_x);
    for i=1:(nstep-1)
        dlength=norm([path_x(i+1),path_y(i+1)]-[path_x(i),path_y(i)]);
        tlength=tlength+dlength;
        if max_dlength<dlength
            max_dlength=dlength;
        end
    end
end

