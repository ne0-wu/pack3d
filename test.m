clc
clear
close all

bunny = Model3D('meshes/ob_chair_gothic.obj');
bunny.ComputeVHACD(20);

m = 3; n = 3; l = 2;
models(1:(m * n * l)) = Model3D;
fig = figure;
for i = 1:m
    for j = 1:n
        for k = 1:l
            idx = (i - 1) * m * l + (j - 1) * l + k;
            models(idx) = copy(bunny);
            models(idx).moveTo([i j k] * 2);
            models(idx).drawConvexHulls(fig);
            hold on
            drawnow
        end
    end
end
axis equal

container = Container(12,10,8,0.2);

fitnessFunction(models,container)