classdef InformedRRTStar
%     start: the starting position of trajectory
%     goal: the end point of trajectory
%     expandDis: the distance of expansion in every step, adjustment for planning_scale_
%     goal_sample_rate: the probability of directly expanding to the end
%     maxIter: the maximum times of iterations, adjustment for the complexity of the graph
%     node_list: the nodes in the path to the end
    properties
        start
        goal
        expandDis
        goal_sample_rate
        maxIter
        node_list
    end
    
    methods(Static)
        function sample = sample_unit_ball()
            randnum = rand(1, 2);
            a = randnum(1, 1);
            b = randnum(1, 2);
            if (b < a)
                t = a;
                a = b;
                b = t;
            end
            sample = [b * cos(2 * pi * a / b); b * sin(2 * pi * a / b)];
        end
    end
    
    methods
        function obj = InformedRRTStar(start, goal)
            obj.start = start;
            obj.goal = goal;
            obj.expandDis = 5;
            obj.goal_sample_rate = 10;
            obj.maxIter = 200;
            obj.node_list = [];
        end
        
% generate a random point when no path has been searched
        function rnd = sample_free_space(obj)
            global planning_scale_
            x_min = planning_scale_.xmin;
            x_max = planning_scale_.xmax;
            y_min = planning_scale_.ymin;
            y_max = planning_scale_.ymax;
            a = unifrnd(0, 100);
            if (a > obj.goal_sample_rate)
                rnd = [x_min + rand(1) * (x_max - x_min), y_min + rand(1) * (y_max - y_min)];
            else
                rnd = [obj.goal(1), obj.goal(2)];
            end
        end

% generate new points in an ellipse
        function rnd = informed_sample(obj, cMax, cMin, xCenter, e_theta)
            if (cMax < Inf)
                xBall = obj.sample_unit_ball();
                rnd = ([cos(e_theta), -sin(e_theta);sin(e_theta), -cos(e_theta)] * [cMax/2, 0;0, cMin/2] * xBall + xCenter)';
            else
                rnd = obj.sample_free_space();
            end
        end

% for the new point, find the nearest Node in node_list
        function min_index = get_nearest_list_index(obj, rnd)
            nodes = obj.node_list;
            min_dis = Inf;
            min_index = 1;
            m = length(nodes);
            for i = 1:m
                temp_dis = (nodes(i, 1).x - rnd(1, 1)) ^ 2 + (nodes(i, 1).y - rnd(1, 2)) ^ 2;
                if (temp_dis < min_dis)
                    min_dis = temp_dis;
                    min_index = i;
                end
            end
        end
 
% generate a new Node based on the new point
        function newNode = get_new_node(obj, theta, n_ind, nearestNode)
            newNode = MyNode(nearestNode.x + obj.expandDis * cos(theta), nearestNode.y + obj.expandDis * sin(theta));
            newNode.cost = nearestNode.cost + obj.expandDis;
            newNode.parent = n_ind;
        end
  
%  A simple function to judge whether there are obstacles between (x1, y1) to (x2, y2)
        function noCollision = check_move(obj, x_1, y_1, x_2, y_2)
            global obstacles_
            from_loc = [x_1, y_1];
            to_loc = [x_2, y_2];
            obs_len = length(obstacles_);
            for i = 1:obs_len
                obs = obstacles_{i};
                if checkObj_point(to_loc, [obs.x; obs.y])
                    noCollision = false;
                    return;
                end
                
                if checkObj_linev(from_loc, to_loc, [obs.x; obs.y])
                    noCollision = false;
                    return;
                end
            end
            noCollision = true;
        end
      
% another check_move method when (x2, y2) is known legal
% each parameters are the same as check_move
        function noCollision = check_move2(obj, x_1, y_1, x_2, y_2)
            global obstacles_
            from_loc = [x_1, y_1];
            to_loc = [x_2, y_2];
            obs_len = length(obstacles_);
            for i = 1:obs_len
                obs = obstacles_{i};
                if checkObj_linev(from_loc, to_loc, [obs.x; obs.y])
                    noCollision = false;
                    return;
                end
            end
            noCollision = true;
        end
  
% find near nodes of newNode
        function near_inds = find_near_nodes(obj, newNode)
            near_inds = zeros(1, obj.maxIter, 'int16');
            nodes = obj.node_list;
            n_node = length(nodes);
            r = 50 * sqrt(log(n_node) / n_node);
            index = 0;
            for i = 1:n_node
                if ((nodes(i, 1).x - newNode.x) ^ 2 + (nodes(i, 1).y - newNode.y) ^ 2 < r ^ 2)
                    index = index + 1;
                    near_inds(1, index) = i;
                end
            end
        end
        
% choose parent for the newNode
        function newNode = choose_parent(obj, Node, near_inds)
            if (near_inds(1, 1) == 0)
                newNode = Node;
            else
                dList = zeros(1, obj.maxIter);
                for i = 1:obj.maxIter
                    dList(1, i) = Inf;
                end
                i = 1;
                while (near_inds(1, i) ~= 0)
                    near_node = obj.node_list(i, 1);
                    if (obj.check_move(near_node.x, near_node.y, Node.x, Node.y))
                        dis = sqrt((near_node.x - Node.x) ^ 2 + (near_node.y - Node.y) ^ 2);
                        dList(1, i) = dis + near_node.cost;
                    end
                    i = i + 1;
                end
                [min_dis, min_Position] = min(dList);
                if (min_dis == Inf)
                    newNode = Node;
                else
                    newNode = Node;
                    newNode.cost = min_dis;
                    newNode.parent = near_inds(min_Position);
                end
            end
        end
     
% get a path through the parent of Nodes
        function path = get_final_course(obj, lastIndex)
            path = [obj.goal(1), obj.goal(2)];
            while (obj.node_list(lastIndex).parent ~= 0)
                node = obj.node_list(lastIndex);
                path = [node.x, node.y;path];
                lastIndex = node.parent;
            end
            path = [obj.start(1), obj.start(2);path];
        end
        
% use informed_rrt_star method to search for path
        function [path, path_length] = informed_rrt_star_search(obj)
            Start = MyNode(obj.start(1), obj.start(2));
            Goal = MyNode(obj.goal(1), obj.goal(2));
            obj.node_list = [obj.node_list ; Start];
            cBest = Inf;
            cMin = sqrt((Start.x - Goal.x) ^ 2 + (Start.y - Goal.y) ^ 2);
            xCenter = [(Start.x + Goal.x) / 2;(Start.y + Goal.y) / 2];
            e_theta = atan2(Goal.y - Start.y, Goal.x - Start.x);
            for i = 1:obj.maxIter
                rnd = obj.informed_sample(cBest, cMin, xCenter, e_theta);
                n_ind = obj.get_nearest_list_index(rnd);
                nearestNode = obj.node_list(n_ind);
                theta = atan2(rnd(2) - nearestNode.y, rnd(1) - nearestNode.x);
                newNode = obj.get_new_node(theta, n_ind, nearestNode);
                noCollision = obj.check_move(nearestNode.x, nearestNode.y, newNode.x, newNode.y);
                if (noCollision)
                    nearInds = obj.find_near_nodes(newNode);
                    newNode = obj.choose_parent(newNode, nearInds);
                    obj.node_list = [obj.node_list ; newNode];
                    d_goal = sqrt((newNode.x - Goal.x) ^ 2 + (newNode.y - Goal.y) ^ 2);
                    if (d_goal < obj.expandDis)
                        if (obj.check_move2(newNode.x, newNode.y, Goal.x, Goal.y))
                            lastIndex = length(obj.node_list);
                            tempPathLen = newNode.cost + d_goal;
                            if (tempPathLen < cBest)
                                path = obj.get_final_course(lastIndex);
                                cBest = tempPathLen;
                            end
                        end
                    end
                end
            end
        path_length = cBest;
        end
    end
end