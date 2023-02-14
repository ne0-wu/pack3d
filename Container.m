classdef Container

    properties
        sizeX
        sizeY
        sizeZ
        minDist
    end

    methods

        function obj = Container(length,width,height,dist)
            obj.sizeX = length;
            obj.sizeY = width;
            obj.sizeZ = height;
            if nargin == 3
                obj.minDist = dist;
            else
                obj.minDist = 0;
            end
        end

        function output = volume(obj)
            output = obj.sizeX * obj.sizeY * obj.sizeZ;
        end

    end

end