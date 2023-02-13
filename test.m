clc
clear
close all

% a = Model3D('meshes/box-thick.obj');
% a.ComputeVHACD;
% b = copy(a);
% 
% b.move([0 80 0])
% detectCollision(a,b,10)
% 
% fig = figure;
% a.drawConvexHulls(fig);
% hold on
% b.drawConvexHulls(fig);
% 
% for i = 1:1
%     b.rotate(30,1);
%     clf(fig)
%     a.drawConvexHulls(fig);
%     hold on
%     b.drawConvexHulls(fig);
%     detectCollision(a,b,10)
%     drawnow
% end

bunny = Model3D('meshes/bunny.obj');
bunny.ComputeVHACD;

m = 5; n = 4; l = 3;
models(1:(m * n * l)) = Model3D;
fig = figure;
for i = 1:m
    for j = 1:n
        for k = 1:l
            idx = (i - 1) * m * n + (j - 1) * n + l;
            models(idx) = copy(bunny);
            models(idx).moveTo([i j k] * 2);
            models(idx).draw(fig);
            hold on
            drawnow
        end
    end
end
axis equal