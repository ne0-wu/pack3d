classdef AABB

    properties
        minX
        maxX
        minY
        maxY
        minZ
        maxZ
    end

    methods

        function obj = AABB(points,pose)
            if nargin == 2
                points = [points ones(size(points,1),1)] * pose';
            end
            obj.minX = min(points(:,1));
            obj.maxX = max(points(:,1));
            obj.minY = min(points(:,2));
            obj.maxY = max(points(:,2));
            obj.minZ = min(points(:,3));
            obj.maxZ = max(points(:,3));
        end

        function obj = add(obj,obj2)
            obj.minX = min(obj.minX,obj2.minX);
            obj.maxX = max(obj.maxX,obj2.maxX);
            obj.minY = min(obj.minY,obj2.minY);
            obj.maxY = max(obj.maxY,obj2.maxY);
            obj.minZ = min(obj.minZ,obj2.minZ);
            obj.maxZ = max(obj.maxZ,obj2.maxZ);
        end

        function [overlapStatus,overlapVolume] = checkOverlap(obj1,obj2)
            overlapStatus = min(obj1.maxX,obj2.maxX) > max(obj1.minX,obj2.minX) && ...
                            min(obj1.maxY,obj2.maxY) > max(obj1.minY,obj2.minY) && ...
                            min(obj1.maxZ,obj2.maxZ) > max(obj1.minZ,obj2.minZ);
            if overlapStatus
                overlapVolume = (min(obj1.maxX,obj2.maxX) - max(obj1.minX,obj2.minX)) * ...
                                (min(obj1.maxY,obj2.maxY) - max(obj1.minY,obj2.minY)) * ...
                                (min(obj1.maxZ,obj2.maxZ) - max(obj1.minZ,obj2.minZ));
            else
                overlapVolume = 0;
            end
        end

        function output = isInside(obj,container)
            output = obj.minX >= 0 && ...
                     obj.minY >= 0 && ...
                     obj.minZ >= 0 && ...
                     obj.maxX <= container.sizeX && ...
                     obj.maxY <= container.sizeY && ...
                     obj.maxZ <= container.sizeZ;
        end

        function output = volume(obj)
            output = (obj.maxX - obj.minX) * ...
                     (obj.maxY - obj.minY) * ...
                     (obj.maxZ - obj.minZ);
        end

        function output = sizeLWH(obj)
            % length, width and height
            output = [obj.maxX - obj.minX ...
                      obj.maxY - obj.minY ...
                      obj.maxZ - obj.minZ];
        end

        function draw(obj)
            colsnbox = collisionBox(obj.maxX - obj.minX, ...
                obj.maxY - obj.minY,obj.maxZ - obj.minZ);
            colsnbox.Pose = [1 0 0 (obj.minX + obj.maxX) / 2; ...
                             0 1 0 (obj.minY + obj.maxY) / 2; ...
                             0 0 1 (obj.minY + obj.maxZ) / 2; ...
                             0 0 0 1                        ];
            show(colsnbox);
        end

    end

end