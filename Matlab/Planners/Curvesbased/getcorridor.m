function [box_list, corridor,time] = getcorridor(prmpath)
global vehicle_kinematics_
psize = size(prmpath,1);
box_list = {};
box_last = inflate_box(prmpath(1,:));
for p = 1:psize
    if is_in_box(box_list,box_last, prmpath(p,:))
        continue;
    end
    box_now = inflate_box(prmpath(p,:));
    flag = delete_box(box_list, box_last,box_now,path);
    if (flag==1)
        
    elseif (flag==2)
        if ~isempty(box_list)
            box_list(end)=[];
        end
        while ~isempty(box_list)
            box_last = box_list{end};
            flag2 = delete_box(box_list, box_last,box_now,path);
            if flag2~=2
                break;
            end
            box_list(end) = [];
        end
        box_list = [box_list;box_now];
        box_last = box_now;
    else
        box_list = [box_list;box_now];
        box_last = box_now;
    end
end
if (size(box_list,1) >1)
    box_list = simplify_box(box_list);
end
segment = size(box_list,1);
corridor = [];
for coi = 1:segment
    xmin = box_list{coi}(1,1);
    xmax = box_list{coi}(2,1);
    ymin = box_list{coi}(3,2);
    ymax = box_list{coi}(2,2);
    corridor = [corridor;[xmin,xmax,ymin,ymax]];
end

center = [];
center = [center;prmpath(1,:)];
for m = 1:segment-1
    center = [center;getoverlapcenter(box_list{m},box_list{m+1})];
end
center = [center;prmpath(psize,:)];
time = [];
for t = 1:segment
    time = [time;norm(center(t+1,:)-center(t,:))/(vehicle_kinematics_.vehicle_v_max*0.3)];
end
end
function res = is_in_box(box_list, box_last, pt)
if isempty(box_list)
    res = 0;
    return
end
if (pt(1)>=box_last(1,1) && ...
        pt(1)<=box_last(2,1) && ...
        pt(2)>=box_last(4,2) && ...
        pt(2)<=box_last(2,2))
    res = 1;
    return
end
res = 0;
return
end

function box = inflate_box(pt)
% p1-------p2
% |         |
% p3-------p4
% box=[x1,y1;
%      x2,y2;
%      x3,y3;
%      x4,y4;
%      cx,xy];
global obstacles_ Nobs planning_scale_
step = 0.3;
max_inflate_iter = 1000;
box = [pt(1)-0.02,pt(2)+0.02;
    pt(1)+0.02,pt(2)+0.02;
    pt(1)-0.02,pt(2)-0.02;
    pt(1)+0.02,pt(2)-0.02;
    pt(1),pt(2)];
has_P2_x = 0;
has_P2_y = 0;
has_P3_x = 0;
has_P3_y = 0;
n = size(path,1);
temp_P2 = box(2,:);
temp_P3 = box(3,:);
for iter = 1:max_inflate_iter
    if (~has_P2_x || ~has_P2_y)
        if(~has_P2_x)
            temp_P2(2) = temp_P2(2) + step;
            temp_P1 = [temp_P3(1),temp_P2(2)];
            % Nob global
            for i = 1:Nobs
                obs = [obstacles_{i}.x;obstacles_{i}.y];
                if (temp_P2(2) >= planning_scale_.ymax || checkObj_linev(temp_P1,temp_P2,obs))
                    has_P2_x = 1;
                    break;
                end
            end
            if (has_P2_x)
                temp_P2(2) = temp_P2(2) - step;
            end
        end
        if(~has_P2_y)
            temp_P2(1) = temp_P2(1) + step;
            temp_P4 = [temp_P2(1),temp_P3(2)];
            % Nob global
            for i = 1:Nobs
                obs = [obstacles_{i}.x;obstacles_{i}.y];
                if (temp_P2(1) >= planning_scale_.xmax || checkObj_linev(temp_P2,temp_P4,obs))
                    has_P2_y = 1;
                    break;
                end
            end
            if (has_P2_y)
                temp_P2(1) = temp_P2(1) - step;
            end
        end
        
    end
    if (~has_P3_x || ~has_P3_y)
        if(~has_P3_x)
            temp_P3(2) = temp_P3(2) - step;
            temp_P4 = [temp_P2(1),temp_P3(2)];
            % Nob global
            for i = 1:Nobs
                obs = [obstacles_{i}.x;obstacles_{i}.y];
                if (temp_P3(2) <= planning_scale_.ymin || checkObj_linev(temp_P3,temp_P4,obs))
                    has_P3_x = 1;
                    break;
                end
            end
            if (has_P3_x)
                temp_P3(2) = temp_P3(2) + step;
            end
        end
        if(~has_P3_y)
            temp_P3(1) = temp_P3(1) - step;
            temp_P1 = [temp_P3(1),temp_P2(2)];
            % Nob global
            for i = 1:Nobs
                obs = [obstacles_{i}.x;obstacles_{i}.y];
                if (temp_P3(1) <= planning_scale_.xmin || checkObj_linev(temp_P3,temp_P1,obs))
                    has_P3_y = 1;
                    break;
                end
            end
            if (has_P3_y)
                temp_P3(1) = temp_P3(1) + step;
            end
        end
    end
    if (has_P2_x + has_P2_y + has_P3_x + has_P3_y == 4)
        break;
    end
end
box(1:4,:)=[temp_P3(1), temp_P2(2);
    temp_P2(1), temp_P2(2);
    temp_P3(1), temp_P3(2);
    temp_P2(1), temp_P3(2)];
end

function result = checkObj_linev(x1,x2,obj)
%% Determine whether the line segment formed by x1 and x2 intersects the obstacle
%% The obstacle here is mainly a polygon with four vertices
result = 1;
% Number of obstacles
[size_n,~] = size(obj);
nobj = size_n/2;
for nobj_i = 1:nobj
    index = (nobj_i-1) * 2 + 1:nobj_i * 2;
    temp_new_obj = obj(index,:);
    %% First determine whether the vertex is inside the obstacle
    result1 = checkObj_point(x1,temp_new_obj);
    result2 = checkObj_point(x2,temp_new_obj);
    if result1 == 0 && result2 == 0 % If none of the line segment endpoints are inside the obstacle
        result = 0;
    else
        result = 1;
        break;
    end
    %% If the vertices are both outside the obstacle, determine whether the two diagonals intersect the line segment
    % Direction of the line segment
    v1 = x2 - x1;
    %% Diagonal1
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
    
    %% Diagonal2
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

function result = checkObj_point(xr,obj)
%% Determine if xr is inside the obstacle
% The obstacle here is mainly a polygon with four vertices
result = 0;
ncorner = 4;
% Calculate the area of the obstacle area_obj and the sum of the four triangles areaarea, if they are equal, it means xr is inside the obstacle
area = 0; area_obj = 0;
% triArea is used to calculate the area of a triangle
for corner_i=1:ncorner
    area = area + triArea(xr,obj(:,corner_i),obj(:,mod(corner_i,ncorner)+1));
end
for corner_i=2:ncorner-1
    area_obj = area_obj + triArea(obj(:,1),obj(:,corner_i),obj(:,mod(corner_i,ncorner)+1));
end
% if the reference point is inside the obstacle, then area = polyarea
if norm(area_obj-area) < 0.01
    result = 1;
end
end
function area = triArea(p1,p2,p3)
a = norm(p1-p2);
b = norm(p1-p3);
c = norm(p2-p3);
half = (a+b+c)/2;
area = sqrt(half*(half-a)*(half-b)*(half-c));
end
function flag = delete_box(box_list, box_last,box_now,path)
n = size(path,1);
has_box_last = 0;
has_box_now = 0;
for i = n:1
    if ~has_box_last
        if path(i,1)>=box_last(1,1)&&...
           path(i,1)<=box_last(2,1)&&...
           path(i,2)>=box_last(4,2)&&...
           path(i,2)<=box_last(2,2)&&...
           path(i,1)<=box_now(1,1)||...
           path(i,1)>=box_now(2,1)||...
           path(i,2)<=box_now(4,2)||...
           path(i,2)>=box_now(2,2)
            has_box_last = 1;
        end
    end
    if ~has_box_now
        if path(i,1)>=box_now(1,1)&&...
           path(i,1)<=box_now(2,1)&&...
           path(i,2)>=box_now(4,2)&&...
           path(i,2)<=box_now(2,2)&&...
           path(i,1)<=box_last(1,1)||...
           path(i,1)>=box_last(2,1)||...
           path(i,2)<=box_last(4,2)||...
           path(i,2)>=box_last(2,2)
            has_box_now = 1;
        end
    end
    if has_box_last && has_box_now
        flag = 0;
        return
    end
end
if  has_box_last && ~has_box_now
    flag = 1;
    return
end
flag = 2;
return
end
function box_list = simplify_box(box_list)
temp = box_list;
n = size(box_list,1);
box_list = {};
idx_old = 1;
box_list = [box_list;temp{idx_old}];
for i = 2:n
    if ~is_overlap(temp{idx_old},temp{i})
        box_list = [box_list;temp{i-1}];
        idx_old = i - 1;
        if (is_overlap(temp{i-1},temp{n}))
            break;
        end
    end
end
box_list = [box_list;temp{n}];
    
end

function res = is_overlap(box_old,box_new)
if (box_new(3,1) >= (box_old(2,1)+1) || ...
        box_new(3,2) >= (box_old(2,2)+1) || ...
        box_old(3,1) >= (box_new(2,1)+1) || ...
        box_old(3,2) >= (box_new(2,2)+1))
    res = 0;
    return
end
res = 1;
return
end

function c = getoverlapcenter(box1,box2)
x_low = max(box1(1,1), box2(1,1));
x_up = min(box1(2,1), box2(2,1));
y_low = max(box1(4,2), box2(4,2));
y_up = min(box1(2,2), box2(2,2));
c = [(x_low + x_up) / 2, (y_low + y_up) / 2];
end