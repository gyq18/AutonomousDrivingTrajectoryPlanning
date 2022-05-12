classdef ActionSet
    %ActionSet Process action primitives。
    %   
    
    properties
        actions
        interpolation_points
    end
    
    methods
         function obj = ActionSet()
             %ActionSet Constructor
             %   
             obj.actions = [];
             obj.interpolation_points = [];
         end
        
        function obj = load_data(obj,interpolation_filename)
                %load_data load action primitives and interpolation points
                %   interpolation_filename: filename of interpolation file.
                action_json_str = '{"0": [[1, 0, 0, 1], [1, 1, 1, 1], [1, 1, 2, 1], [1, -1, 6, 1], [1, -1, 7, 1], [-1, 0, 0, 0], [-1, 1, 6, 0], [-1, 1, 7, 0], [-1, -1, 1, 0], [-1, -1, 2, 0]], "1": [[1, 1, 0, 1], [1, 1, 1, 1], [1, 1, 2, 1], [0, 1, 3, 1], [1, 0, 7, 1], [-1, -1, 0, 0], [-1, -1, 1, 0], [-1, -1, 2, 0], [0, -1, 3, 0], [-1, 0, 7, 0]], "2": [[0, 1, 2, 1], [-1, 1, 3, 1], [-1, 1, 4, 1], [1, 1, 0, 1], [1, 1, 1, 1], [0, -1, 2, 0], [-1, -1, 0, 0], [-1, -1, 1, 0], [1, -1, 3, 0], [1, -1, 4, 0]], "3": [[-1, 1, 2, 1], [-1, 1, 3, 1], [-1, 1, 4, 1], [-1, 0, 5, 1], [0, 1, 1, 1], [1, -1, 2, 0], [1, -1, 3, 0], [1, -1, 4, 0], [1, 0, 5, 0], [0, -1, 1, 0]], "4": [[-1, 0, 4, 1], [-1, -1, 5, 1], [-1, -1, 6, 1], [-1, 1, 2, 1], [-1, 1, 3, 1], [1, 0, 4, 0], [1, -1, 2, 0], [1, -1, 3, 0], [1, 1, 5, 0], [1, 1, 6, 0]], "5": [[-1, -1, 4, 1], [-1, -1, 5, 1], [-1, -1, 6, 1], [0, -1, 7, 1], [-1, 0, 3, 1], [1, 1, 4, 0], [1, 1, 5, 0], [1, 1, 6, 0], [0, 1, 7, 0], [1, 0, 3, 0]], "6": [[0, -1, 6, 1], [1, -1, 7, 1], [1, -1, 0, 1], [-1, -1, 4, 1], [-1, -1, 5, 1], [0, 1, 6, 0], [1, 1, 4, 0], [1, 1, 5, 0], [-1, 1, 7, 0], [-1, 1, 0, 0]], "7": [[1, -1, 6, 1], [1, -1, 7, 1], [1, -1, 0, 1], [1, 0, 1, 1], [0, -1, 5, 1], [-1, 1, 6, 0], [-1, 1, 7, 0], [-1, 1, 0, 0], [-1, 0, 1, 0], [0, 1, 5, 0]]}';
                actions_ = jsondecode(action_json_str);
                obj.actions = actions_;
    
                file = fopen(interpolation_filename, "r");
                C = textscan(file, '%s', 'delimiter', '\n');
                intpl_json_str = char(C{1});
                interpolation_points_ = jsondecode(intpl_json_str);
                obj.interpolation_points = interpolation_points_;
        end

    
        function actions =  get_actions(obj, position, direction, x_len, y_len)
                %get_actions Get all valid action in current position.
                %   position: current position
                %   direction: current direction.
                %   x_len: x-axis length of the graph.
                %   y_len: y-axis length of the graph.
            action_list = obj.actions.("x" + sprintf("%d", direction));
            actions = obj.action_clean(action_list, position, x_len, y_len);
        end

        function actions = action_clean(~, action_list, position, x_len, y_len)
            %action_clean Delete all invalid action.
            %   action_list: possible action list
            %   position: current position
            %   x_len: x-axis length of the graph.
            %   y_len: y-axis length of the graph.

            result_actions = {};
            for action = action_list'
                if position(1) == 0
                    if (action(1) == -1) || ((action(1) == 0) && ((action(3)==1) || action(3) == 7))
                        continue;
                    end
                elseif position(1) == x_len - 1
                    if (action(1) == 1) || ((action(1) == 0) && ((action(3)==3) || action(3) == 5))
                        continue;
                    end
                end

%                 check y
                if position(2) == 0
                    if (action(2) == -1) || ((action(2) == 0) && ((action(3)==1) || action(3) == 3))
                        continue;
                    end
                elseif position(2) == y_len - 1
                    if (action(2) == 1) || ((action(2) == 0) && ((action(3)==4) || action(3) == 7))
                        continue;
                    end
                end
                result_actions{length(result_actions)+1} = action;
            end
            actions = result_actions;
        end

        function [x, y, theta] = sample_points(obj, start_pos, start_direction, end_pos, end_direction)
            %sample_points make interpolation for all actions. Make
            %trajectory smooth
            %   start_pos: start position
            %   start_direction: the direction when in start position
            %   end_pos: end position
            %   end_direction: the end direction.
            
            delta_pos = end_pos - start_pos;
            action_list = obj.actions.("x" + sprintf("%d", start_direction));
            
            index = 0;
            for action = action_list'
                if (action(1) == delta_pos(1)) && (action(2) == delta_pos(2)) && (action(3) == end_direction)
                    break;
                end
                index = index + 1;
            end
            xyt_result = obj.interpolation_points.("x" + sprintf("%d", start_direction)).("x" + sprintf("%d", index));
            x_list = xyt_result.x;
            y_list = xyt_result.y;
            theta_list = xyt_result.theta;

            x_list = x_list + double(start_pos(1));
            y_list = y_list + double(start_pos(2));

            x = x_list;
            y = y_list;
            theta = theta_list;

        end

    end


end

