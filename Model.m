classdef Model < handle
    % 3D models to pack
    properties
        Vertices
        Triangles

        % homogeneous matrix
        Pose = eye(4)

        % object is decomposed into convex collision meshes
        convexDecomp = collisionMesh([0 0 0])

        % axis-aligned bounding box
        BoundingBox
        
        % (usually random) color, for visualization
        Color
    end

    methods

        function obj = Model(arg1, arg2)
            switch nargin
                case 0  % no input
                    obj.Vertices = [0 0 0;
                                    1 0 0;
                                    0 1 0;
                                    0 0 1];
                    obj.Triangles = [1 2 3;
                                     1 3 4;
                                     1 4 2;
                                     4 3 2];
                case 1 % input is filename
                    [obj.Vertices, obj.Triangles] = readObj(arg1);
                case 2 % input is vertices and triangles
                    obj.Vertices = arg1;
                    obj.Triangles = arg2;
            end
            
            % obj.ComputeVHACD(8);   % comvex decomposition

            % move barycenter to 0 for rotation
            barycenter = mean(obj.Vertices);
            obj.Vertices = obj.Vertices - barycenter; 
            obj.BoundingBox = AABB(obj.Vertices);
            obj.Color = rand(1, 3); % random color by default
        end

        function cp = copy(obj)
            cp = Model(obj.Vertices, obj.Triangles);
            cp.convexDecomp(1:length(obj.convexDecomp)) = ... 
                collisionMesh([0 0 0]); % initialize the array
            for i = 1:length(obj.convexDecomp)
                cp.convexDecomp(i) = ...
                    collisionMesh(obj.convexDecomp(i).Vertices);
            end
            cp.setPose(obj.Pose)
            cp.BoundingBox = AABB(cp.Vertices, cp.Pose);
        end

        %% convex decomposition with VHACD
        function ComputeVHACD(obj, maxconvexDecomp)
            if nargin == 1
                maxconvexDecomp = 16;
            end

            % compute convex decomposition using C++ V-HACD library
            [pos, offset] = mexVHACD(obj.Vertices, obj.Triangles - 1, maxconvexDecomp);

            obj.convexDecomp(1:(size(offset, 2) - 1)) = collisionMesh([0 0 0]);
            for i = 1:(size(offset, 2) - 1)
                obj.convexDecomp(i) = ...
                    collisionMesh(pos((offset(i) + 1):offset(i + 1), :));
            end
        end

        %% draw
        % draw the mesh
        function fig = draw(obj, fig)
            if nargin == 1, fig = figure; end

            v = [obj.Vertices ones(size(obj.Vertices, 1), 1)] * obj.Pose';
            trimesh(obj.Triangles, v(:, 1), v(:, 2), v(:, 3), ...
                'EdgeColor', [0 0 0], 'FaceColor', obj.Color);
            axis equal
        end

        % draw the convex hulls produced by VHACD
        function fig = drawConvexDecomp(obj, fig)
            if nargin == 1, fig = figure;  end

            for i = 1:length(obj.convexDecomp)
                show(obj.convexDecomp(i));
                hold on
            end
            hold off
            axis equal
        end

        %% deal with the pose of the model
        % get position of the model
        function pos = position(obj)
            pos = obj.Pose(1:3, 4)';
        end

        % set the object's pose, which is a homogeneous matrix
        function setPose(obj, pose)
            obj.Pose = pose;
            for i = 1:length(obj.convexDecomp)
                obj.convexDecomp(i).Pose = obj.Pose;
            end
            obj.BoundingBox = AABB(obj.Vertices, obj.Pose);
        end

        % perform an affine transform to the object
        function affineTrsfm(obj, aftMatrix)
            obj.setPose(aftMatrix * obj.Pose);
        end

        % move model from x to x+posDiff
        function move(obj, posDiff)
            trslMatrix = eye(4);
            trslMatrix(1:3, 4) = posDiff;
            obj.setPose(trslMatrix * obj.Pose);
        end

        % move model to desired position
        function moveTo(obj, position)
            newPose = obj.Pose;
            newPose(1:3, 4) = position;
%             switch size(position, 1)
%                 case 3
%                     newPose(1:3, 4) = position;
%                 case 1
%                     newPose(1:3, 4) = position';
%             end
            obj.setPose(newPose);
        end

        % rotate the object around its barycenter
        function rotate(obj, deltaTheta, dim)
            switch (dim)
                case 1, rotm = rotx(deltaTheta);
                case 2, rotm = roty(deltaTheta);
                case 3, rotm = rotz(deltaTheta);
            end

            newPose = obj.Pose;
            newPose(1:3, 1:3) = rotm * newPose(1:3, 1:3);
            obj.setPose(newPose);
        end

        %% check the collision status between 2 objects
        function collisionStatus = checkModelCollision(obj1, obj2, minDist)
            % we suppose that obj1 collide with obj2 if they overlap
            % if input includes the third argument, we suppose that obj1
            % collide with obj2 if dist(obj1,obj2) < minDist
            if nargin == 2
                minDist = 0;
            end

            collisionStatus = false;
            for ch1 = obj1.convexDecomp
                for ch2 = obj2.convexDecomp
                    [overlap, dist] = ...
                        checkCollision(ch1, ch2);
                    if overlap || dist < minDist
                        collisionStatus = true;
                        return
                    end
                end
            end
%             for i = 1:length(obj1.convexDecomp)
%                 for j = 1:length(obj2.convexDecomp)
%                     [overlap, dist] = ...
%                         checkCollision(obj1.convexDecomp(i), ...
%                                        obj2.convexDecomp(j));
%                     if overlap || (nargin == 3 && dist < minDist)
%                         collisionStatus = true;
%                         return
%                     end
%                 end
%             end
        end

    end

end
