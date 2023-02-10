classdef TriMesh3D < handle
% 3D triangular mesh

    properties
        Vertices
        Triangles
    end

    methods

        function obj = TriMesh3D(arg1,arg2)
            switch nargin
                case 1
                    filename = arg1;
                    [obj.Vertices,obj.Triangles] = readObj(filename);
                case 2
                    obj.Vertices = arg1;
                    obj.Triangles = arg2;
            end
        end

        function fig = draw(obj)
            fig = figure;
            trimesh(obj.Triangles,obj.Vertices(:,1),obj.Vertices(:,2),obj.Vertices(:,3));
            axis equal
        end

    end
    
end