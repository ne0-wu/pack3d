clear
close all

model = Model3D('meshes/ob_chair_gothic.obj');
model.ComputeVHACD(8);

m = 3; n = 3; l = 1;
models(1:(m * n * l)) = Model3D;
for i = 1:m
    for j = 1:n
        for k = 1:l
            idx = (i - 1) * m * l + (j - 1) * l + k;
            models(idx) = copy(model);
            models(idx).moveTo([3 3 3]);
        end
    end
end

container = Container(6,6,6);

simulatedAnnealing(models,container);

% fig = figure;
% for i = 1:length(models)
%     models(i).drawConvexHulls(fig);
%     hold on
%     drawnow
% end