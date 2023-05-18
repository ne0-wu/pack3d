% clear
% close all

% clear packing models
% 
% chair = Model('meshes/chair.obj');
% chair.ComputeVHACD(5);

% casterSwivel = Model('meshes/caster-swivel.obj');
% casterSwivel.Vertices = casterSwivel.Vertices / 15;
% casterSwivel.ComputeVHACD(6);

% chair.drawConvexDecomp

% N = 20;
% 
% models(1:N) = Model();
% for i = 1:N, models(i) = copy(chair); end

% for i = 11:15, models(i) = copy(casterSwivel); end

% m = 3; n = 3; l = 1;
% models(1:(m * n * l)) = Model;
% for i = 1:m
%     for j = 1:n
%         for k = 1:l
%             idx = (i - 1) * m * l + (j - 1) * l + k;
%             models(idx) = copy(model01);
%             models(idx).moveTo([3 3 3]);
%         end
%     end
% end

% wc = PackingSA(3, 3, 3, 0);
% wc.appendModel(models);
% wc.pack(2e4);

% optimal
packing = PackingBL(3, 3, 3, 0.1);
packing.appendModel(models);

for i = 2:2:length(models)
    packing.models(i).rotate(180, 3);
end

for i = 17:20
    packing.models(i).rotate(90, 3);
end

orientation = zeros(1, packing.numModels, 3);
orientation(1, 2:2:length(models), 3) = 2;
orientation(1, 17:20, 3) = orientation(1, 17:20, 3) + 1;
% packing.bottomLeft(1:packing.numModels, orientation)
packing.fitnessFunc(1:20, orientation)

%% genetic
% packing = PackingGeneticBL(3, 3, 8, 0.1);
% packing.appendModel(models);
% 
% packing.pack(50, 30);