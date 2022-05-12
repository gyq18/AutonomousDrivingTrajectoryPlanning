classdef MyNode
    properties
        x
        y
        cost
        parent
    end
    
    methods
        function obj = MyNode(x, y)
            obj.x = x;
            obj.y = y;
            obj.cost = 0;
            obj.parent = 0;
        end
    end
end