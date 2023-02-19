classdef Container

    properties
        sizeX
        sizeY
        sizeZ
        minDist
        aabb
    end

    methods

        function obj = Container(length,width,height,dist)
            obj.sizeX = length;
            obj.sizeY = width;
            obj.sizeZ = height;
            if nargin == 4
                obj.minDist = dist;
            else
                obj.minDist = 0;
            end
            obj.aabb = AABB(0,obj.sizeX,0,obj.sizeY,0,obj.sizeZ);
        end

        function output = volume(obj)
            output = obj.sizeX * obj.sizeY * obj.sizeZ;
        end

    end

end