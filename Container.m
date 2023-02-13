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
            obj.minDist = dist;
        end

        function output = volume(obj)
            output = obj.sizeX * obj.sizeY * obj.sizeZ;
        end

    end

end