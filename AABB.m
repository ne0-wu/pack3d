classdef AABB

    properties
        % [min x, max x, min y, max y, min z, max z]
        box
    end

    methods

        function obj = AABB(arg1,arg2)
            obj.box = zeros(1, 6);
            if nargin == 0
                return
            end
            if size(arg1, 2) == 3    % input is points
                points = arg1;
                if nargin == 2       % input contains a pose matrix
                    pose = arg2;
                    points = [points ones(size(points,1),1)] * pose';
                end
                obj.box(1) = min(points(:,1));
                obj.box(2) = max(points(:,1));
                obj.box(3) = min(points(:,2));
                obj.box(4) = max(points(:,2));
                obj.box(5) = min(points(:,3));
                obj.box(6) = max(points(:,3));
            else    % input is [min x, max x, ...]
                obj.box = arg1;
            end
        end

        function out = minX(obj), out = obj.box(1); end
        function out = maxX(obj), out = obj.box(2); end
        function out = minY(obj), out = obj.box(3); end
        function out = maxY(obj), out = obj.box(4); end
        function out = minZ(obj), out = obj.box(5); end
        function out = maxZ(obj), out = obj.box(6); end

        % the AABB of two AABBs
        function obj = aabbUnion(obj1,obj2)
            obj = AABB;
            obj.box([1 3 5]) = min(obj1.box([1 3 5]),obj2.box([1 3 5]));
            obj.box([2 4 6]) = max(obj1.box([2 4 6]),obj2.box([2 4 6]));
        end

        function [overlapStatus, overlapVolume] = checkAabbOverlap(obj1,obj2)
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

        % check if the box is inside the container
        function output = isInside(obj, containerSize)
            output = obj.minX >= 0 && ...
                     obj.minY >= 0 && ...
                     obj.minZ >= 0 && ...
                     obj.maxX <= containerSize(1) && ...
                     obj.maxY <= containerSize(2) && ...
                     obj.maxZ <= containerSize(3);
        end

        % get the volume of the box
        function output = volume(obj)
            output = (obj.maxX - obj.minX) * ...
                     (obj.maxY - obj.minY) * ...
                     (obj.maxZ - obj.minZ);
        end

        % get the length, width and height of the box
        function output = boxSize(obj)
            output = [obj.maxX - obj.minX ...
                      obj.maxY - obj.minY ...
                      obj.maxZ - obj.minZ];
        end

        function draw(obj)
            clsnbox = collisionBox(obj.maxX - obj.minX, ...
                obj.maxY - obj.minY,obj.maxZ - obj.minZ);
            clsnbox.Pose = [1 0 0 (obj.minX + obj.maxX) / 2; ...
                            0 1 0 (obj.minY + obj.maxY) / 2; ...
                            0 0 1 (obj.minY + obj.maxZ) / 2; ...
                            0 0 0 1                        ];
            show(clsnbox);
        end

    end

end