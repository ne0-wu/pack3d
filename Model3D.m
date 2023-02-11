classdef Model3D
% 3D models to pack
    
    properties
        Vertices
        Triangles
        Pose = eye(4)
        ConvexHulls = collisionMesh([0 0 0])
        BoundingBox
    end
    
    methods

        function obj = Model3D(arg1,arg2)
%             switch nargin
%                 case 1
%                     args{1} = arg1;
%                 case 2
%                     args{1} = arg1;
%                     args{2} = arg2;
%             end
%             obj@TriMesh3D(args{:});
            switch nargin
                case 1  % input is filename
                    [obj.Vertices,obj.Triangles] = readObj(arg1);
                case 2  % input is vertices and triangles
                    obj.Vertices = arg1;
                    obj.Triangles = arg2;
            end
            
            obj.ComputeVHACD;
            obj.BoundingBox = AABB(obj.Vertices);
        end

        function cp = copy(obj)
            cp = Model3D(obj.Vertices,obj.Triangles);
            cp.Pose = obj.Pose;
            cp.convexHulls(1:size(obj.ConvexHulls)) = collisionMesh([0 0 0]);
            for i = 1:length(obj.ConvexHulls)
                cp.convexHulls(i) = ...
                    collisionMesh(obj.ConvexHulls(i).Vertices);
            end
            cp.aabb = AABB(cp.Vertices,cp.Pose);
        end

        function ComputeVHACD(obj)
            [pos,offset] = mexVHACD(obj.Vertices,obj.Triangles - 1);
            obj.ConvexHulls(1:(size(offset,2) - 1)) = collisionMesh([0 0 0]);
            for i = 1:(size(offset,2) - 1)
                obj.ConvexHulls(i) = ...
                    collisionMesh(pos((offset(i) + 1):offset(i + 1),:));
            end
        end

        function fig = drawConvexHulls(obj,fig)
            if nargin == 1
                fig = figure;
            end
            for i = 1:length(obj.ConvexHulls)
                show(obj.ConvexHulls(i));
                hold on
            end
            axis equal
        end

        function setPose(obj,pose)
            obj.Pose = pose;
            for i = 1:length(obj.ConvexHulls)
                obj.ConvexHulls(i).Pose = obj.Pose;
            end
        end

        function affineTrsfm(obj,trans)
            obj.Pose = trans * obj.Pose;
            for i = 1:length(obj.ConvexHulls)
                obj.ConvexHulls(i).Pose = obj.Pose;
            end
            obj.BoundingBox = AABB(obj.Vertices,obj.Pose);
        end

        function move(obj,deltaR)
            deltaX = deltaR(1);
            deltaY = deltaR(2);
            deltaZ = deltaR(3);
            obj.affineTrsfm([1 0 0 deltaX; ...
                             0 1 0 deltaY; ...
                             0 0 1 deltaZ; ...
                             0 0 0 1]);
        end

        function rotate(obj,deltaTheta,dim)
%             switch(dim)
%                 case 1
%                     rot = [1 0               0                0; ...
%                            0 cos(deltaTheta) -sin(deltaTheta) 0; ...
%                            0 sin(deltaTheta) cos(deltaTheta)  0; ...
%                            0 0               0                1];
%                 case 2
%                     rot = [cos(deltaTheta)  0 sin(deltaTheta) 0; ...
%                            0                1 0               0; ...
%                            -sin(deltaTheta) 0 cos(deltaTheta) 0; ...
%                            0                0 0               1];
%                 case 3
%                     rot = [cos(deltaTheta) -sin(deltaTheta) 0 0; ...
%                            sin(deltaTheta) cos(deltaTheta)  0 0; ...
%                            0               0                1 0; ...
%                            0               0                0 1];
%             end
            switch(dim)
                case 1
                    rotm = rotx(deltaTheta);
                case 2
                    rotm = roty(deltaTheta);
                case 3
                    rotm = rotz(deltaTheta);
            end
            obj.affineTrsfm(rotm2tform(rotm));
        end

        function [collisionStatus,minDist] = detectCollision(obj1,obj2)
            [~,minDist] = checkCollision(obj1.ConvexHulls(1),obj2.convexHulls(1));
            for i = 1:length(obj1.ConvexHulls)
                for j = i:length(obj2.convexHulls)
                    [collisionStatus,spdist] = ...
                        checkCollision(obj1.ConvexHulls(i),obj2.convexHulls(j));
                    if collisionStatus
                        minDist = 0;
                        return
                    end
                    if spdist < minDist
                        minDist = spdist;
                    end
                end
            end
        end

    end
    
end