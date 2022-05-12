classdef StateLatticeGraph
    %StateLatticeGraph The State Lattice graph class
    %   solve the program in this graph
    
    properties
        direction_num
        x_length
        y_length
        node_num
        action_set
        nodes
        edges

    end
    
    methods
        function obj = StateLatticeGraph(x_len, y_len, action_set)
            %StateLatticeGraph  Constructor
            %   x_len: the x-axis node number of the graph.
            %   y_len: the y-axis node number of the graph.
            %   action_set: the ActionSet with data loaded.
            obj.x_length = x_len;
            obj.y_length = y_len;
            obj.direction_num = 8;
            obj.node_num = x_len * y_len * obj.direction_num;
            obj.action_set = action_set;
            obj.nodes = {};
            obj.edges = {};

            obj = obj.build_graph();
        end

        function [pos, direction] = index2position(obj, index)
%             Note that matlab start indices from 1 ...
%           The index and the result are from zero for now
            x = idivide(index, (obj.y_length * obj.direction_num));
            y = idivide(index - x * obj.y_length * obj.direction_num, obj.direction_num);
            direction = index - x * obj.y_length * obj.direction_num - y * obj.direction_num;
            pos = [x, y];

        end

        function index = position2index(obj, position, direction)
            %position2index Convert position/direction to index.
            x = position(1);
            y = position(2);
            index = x * obj.y_length * obj.direction_num + y * obj.direction_num + direction;
            if index == -2
                disp("gg")
            end
        end

        function obj = build_graph(obj)
%             LatticeNode is defined as cell{pos(2), direction}
%           construct nodes
            for i = 1:obj.node_num
                [pos, direction] = obj.index2position(i-1);
                obj.nodes{length(obj.nodes)+1} = {pos, direction};
            end
            
%             LatticeEdge is defined as cell{adj_index, edge_cost}
            for i = 1:obj.node_num
                [current_pos, direction] = obj.index2position(i - 1);
                direction_actions = obj.action_set.get_actions(current_pos, direction, obj.x_length, obj.y_length);
                
                action_edges = {};
                for action = direction_actions
                    action = action{1};
                    % calculate the adjacent node index
                    next_position = [current_pos(1) + action(1), current_pos(2) + action(2)];
                    next_direction = action(3);
                    adj_index = obj.position2index(next_position, next_direction);
                    action_edges{length(action_edges)+1} = {adj_index, action2cost(direction, action)};
                end
                obj.edges{length(obj.edges)+1} = action_edges;
            end

            
        end
    end
end

function cost = action2cost(direction, action)
    %action2cost Calculate the cost of given action.
    delta_x = abs(action(1));
    delta_y = abs(action(2));
    x = max([delta_x, delta_y]);
    y = min([delta_x, delta_y]);
    if mod(direction, 2) == 0
        if x == 9
            cost = 10.734;
        elseif x == 11
            cost = 13.263;
        elseif x == 12
            cost = 15.109;
        elseif x == 7
            if y == 3
                cost = 11.498;
            else
                cost = 16.616;
            end
        else
            cost = 1;
        end
    else
        if y == 6
            cost = 9.992;
        elseif y == 7
            if x == 9
                cost = 12.374;
            else
                cost = 13.728;
            end
        elseif y == 3
            cost = 11.498;
        elseif y == 0
            cost = 16.603;
        else
            cost = 1.414;
        end
        
    end

%   use this factor to punishment going backward.
%     coff = 1;
%     if ( direction == next_direction)
%         cost = coff * hypot(delta_x, delta_y);
%     else 
%         cost = coff * pi / 2;
%     end
    
end


