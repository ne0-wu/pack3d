clear
close all

model01 = Model('meshes/ob_chair_gothic.obj');
model01.ComputeVHACD(8);

models(1:20) = Model;
for i = 1:length(models)
    models(i) = copy(model01);
end

% m = 3; n = 3; l = 1;
% models(1:(m * n * l)) = Model;
% for i = 1:m
%     for j = 1:n
%         for k = 1:l
%             idx = (i - 1) * m * l + (j - 1) * l + k;
%             models(idx) = copy(model);
%             models(idx).moveTo([3 3 3]);
%         end
%     end
% end

wc = WorkingChamber(3, 3, 3, 0);
wc.appendModel(models);
wc.simulatedAnnealing(2e4);