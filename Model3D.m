classdef Model3D < handle
% 3D models to pack
    
    properties
        Vertices
        Triangles
        Pose = eye(4)   % homogeneous matrix
        ConvexHulls = collisionMesh([0 0 0])    % object is decomposed into convex collision meshes
        BoundingBox     % axis-aligned bounding box
    end
    
    methods

        function obj = Model3D(arg1,arg2)
            switch nargin
                case 0
                    obj.Vertices = [0 0 0;
                                    1 0 0;
                                    0 1 0;
                                    0 0 1];
                    obj.Triangles = [1 2 3;
                                     1 3 4;
                                     1 4 2;
                                     4 3 2];
                case 1  % input is filename
                    [obj.Vertices,obj.Triangles] = readObj(arg1);
                case 2  % input is vertices and triangles
                    obj.Vertices = arg1;
                    obj.Triangles = arg2;
            end
            barycenter = mean(obj.Vertices);
            obj.Vertices = obj.Vertices - barycenter;   % move barycenter to 0 to simplify rotation
            obj.BoundingBox = AABB(obj.Vertices);
        end

        function cp = copy(obj)
            cp = Model3D(obj.Vertices,obj.Triangles);
            cp.Pose = obj.Pose;
            cp.ConvexHulls(1:size(obj.ConvexHulls)) = collisionMesh([0 0 0]);
            for i = 1:length(obj.ConvexHulls)
                cp.ConvexHulls(i) = ...
                    collisionMesh(obj.ConvexHulls(i).Vertices);
            end
            cp.BoundingBox = AABB(cp.Vertices,cp.Pose);
        end

        function ComputeVHACD(obj)
            % compute convex decomposition using C++ V-HACD library
            [pos,offset] = mexVHACD(obj.Vertices,obj.Triangles - 1);
            obj.ConvexHulls(1:(size(offset,2) - 1)) = collisionMesh([0 0 0]);
            for i = 1:(size(offset,2) - 1)
                obj.ConvexHulls(i) = ...
                    collisionMesh(pos((offset(i) + 1):offset(i + 1),:));
            end
        end

        function fig = draw(obj,fig)
            if nargin == 1
                fig = figure;
            end
            v = [obj.Vertices ones(size(obj.Vertices,1),1)] * obj.Pose';
            trimesh(obj.Triangles,v(:,1),v(:,2),v(:,3));
            axis equal
        end

        function fig = drawConvexHulls(obj,fig)
            if nargin == 1
                fig = figure;
            end
            for i = 1:length(obj.ConvexHulls)
                show(obj.ConvexHulls(i));
                hold on
            end
            hold off
            axis equal
        end

        function setPose(obj,pose)
            % directly change the object's pose
            obj.Pose = pose;
            for i = 1:length(obj.ConvexHulls)
                obj.ConvexHulls(i).Pose = obj.Pose;
            end
            obj.BoundingBox = AABB(obj.Vertices,obj.Pose);
        end

        function affineTrsfm(obj,aftMatrix)
            % perform an affine transform to the object
            obj.setPose(aftMatrix * obj.Pose);
        end

        function move(obj,deltaR)
            obj.affineTrsfm([1 0 0 deltaR(1); ...
                             0 1 0 deltaR(2); ...
                             0 0 1 deltaR(3); ...
                             0 0 0 1        ]);
        end

        function moveTo(obj,position)
            newPose = obj.Pose;
            switch size(position,1)
                case 3
                    newPose(1:3,4) = position;
                case 1
                    newPose(1:3,4) = position';
            end
            obj.setPose(newPose);
        end

        function rotate(obj,deltaTheta,dim)
            % rotate the object around its barycenter
            switch(dim)
                case 1
                    rotm = rotx(deltaTheta);
                case 2
                    rotm = roty(deltaTheta);
                case 3
                    rotm = rotz(deltaTheta);
            end
            newPose = obj.Pose;
            newPose(1:3,1:3) = rotm * newPose(1:3,1:3);
            obj.setPose(newPose);
        end

        function collisionStatus = detectCollision(obj1,obj2,minDist)
            % check the collision status between 2 objects
            % we suppose that obj1 collide with obj2 if they overlap
            % if input includes the third argument, we suppose that obj1
            % collide with obj2 if dist(obj1,obj2) < minDist
            collisionStatus = false;
            for i = 1:length(obj1.ConvexHulls)
                for j = 1:length(obj2.ConvexHulls)
                    [overlap,dist] = ...
                        checkCollision(obj1.ConvexHulls(i),obj2.ConvexHulls(j));
                    if overlap || (nargin == 3 && dist < minDist)
                        collisionStatus = true;
                        return
                    end
                end
            end
        end

        function output = isInside(obj,container)
            output = obj.BoundingBox.minX >= 0 && ...
                     obj.BoundingBox.minY >= 0 && ...
                     obj.BoundingBox.minZ >= 0 && ...
                     obj.BoundingBox.maxX <= container.sizeX && ...
                     obj.BoundingBox.minX <= container.sizeY && ...
                     obj.BoundingBox.minX <= container.sizeZ;
        end

    end
    
end