clear
close all

% a = Model3D('meshes/box-thick.obj');
% a.ComputeVHACD;
% b = copy(a);
% b.move([0 100 0])
% b.rotate(45,1);
% [~,~] = detectCollision(a,b)
% 
% fig = figure;
% a.drawConvexHulls(fig);
% hold on
% b.drawConvexHulls(fig);

n = 10;
pts = zeros(n,3);
for k = 1:n
    ph = 2*pi*rand(1);
    th = pi*rand(1);
    pts(k,:) = [cos(th)*sin(ph) sin(th)*sin(ph) cos(ph)];
end

aabb = AABB(pts)
aabb2 = AABB(pts)