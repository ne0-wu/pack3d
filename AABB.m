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

        function obj = AABB(points)
            obj.minX = min(points(:,1));
            obj.maxX = max(points(:,1));
            obj.minY = min(points(:,2));
            obj.maxY = max(points(:,2));
            obj.minZ = min(points(:,3));
            obj.maxZ = max(points(:,3));
        end

        function result = detectCollision(obj1,obj2)
            result = min(obj1.maxX,obj2.maxX) > max(obj1.minX,obj2.minX) && ...
                     min(obj1.maxY,obj2.maxY) > max(obj1.minY,obj2.minY) && ...
                     min(obj1.maxZ,obj2.maxZ) > max(obj1.minZ,obj2.minZ);
        end

        function draw
            show()
        end

    end

end