classdef Container

    properties
        sizeX
        sizeY
        sizeZ
    end

    methods

        function obj = Container(length,width,height)
            obj.sizeX = length;
            obj.sizeY = width;
            obj.sizeZ = height;
        end

    end

end