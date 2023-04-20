clear
close all

chair = Model('meshes/ob_chair_gothic.obj');
chair.ComputeVHACD(6);

models(1:10) = Model();
for i = 1:length(models)
    models(i) = copy(chair);
end

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

% packing = PackingBL(3, 3, 3, 0.1);
% packing.appendModel(models);
% 
% for i = 2:2:length(models)
%     packing.models(i).rotate(180, 3);
% end
% 
% for i = 17:20
%     packing.models(i).rotate(90, 3);
% end

packing = PackingGeneticBL(2, 2, 8, 0.1);
packing.appendModel(models);

disp('packing start')

packing.pack(50, 50);