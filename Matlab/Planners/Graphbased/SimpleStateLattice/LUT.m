classdef LUT
    %LUT Class of lookuptable
    %   
    
    properties
        edge_length
        lut
        direction_num
    end
    
    methods
        function obj = LUT(filename)
            %LUT Constructor
            %   filename: the filename of lookup table
            obj.direction_num = 8;
            file = fopen(filename);
            C = textscan(file, "%s", 'Delimiter','\n');
            data_str = char(C{1});
            json_obj = jsondecode(data_str);
            obj.edge_length = json_obj.edge_length;
            
            lut = {};
            for i = 0:7
                index_str = "x" + sprintf("%d", i);
                lut{length(lut)+1} = json_obj.(index_str);
            end
            obj.lut = lut;

        end
        
        function result = look_up(obj, start_direction, to_pos, to_direction)
            %look_up look up the cost of specific position/direction in
            %look up table
            %   start_direction: the start direction. the start position is
            %   [0,0]
            %   to_pos: the desitabtion position.
            %   to_direction: the destinate direction.
            index = to_pos(1) * obj.edge_length * obj.direction_num + to_pos(2) * obj.direction_num + to_direction;
            result_table = obj.lut{start_direction+1};
            result = result_table(index + 1);
        end
    end
end

