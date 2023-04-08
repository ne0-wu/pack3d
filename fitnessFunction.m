function [fitness,isFeasible] = fitnessFunction(models,container)

%% check if the current status is a feasible solution
isFeasible = true;

% check if all objects are inside the container
allModelsBB = models(1).BoundingBox;
for i = 2:length(models)
    allModelsBB = allModelsBB.aabbUnion(models(i).BoundingBox);
end
if ~allModelsBB.isInside(container)
    isFeasible = false;
end

overallBB = allModelsBB.aabbUnion(container.aabb);

% for iModel = models
%     if ~iModel.BoundingBox.isInside(container)
%         isFeasible = false;
%         break
%     end
% end

% check if objects collide with each other
overlapVolume = zeros(length(models));
for i = 1:length(models)
    for j = [1:(i-1) (i+1):length(models)]
        if detectCollision(models(i),models(j),container.minDist)
            isFeasible = false;
            [~,overlapVolume(i,j)] = checkOverlap(models(i).BoundingBox,models(j).BoundingBox);
        end
    end
end

%% calculate the fitness function
wA = 1000;
wO = 100;

% maxHeight = allModelsBB.maxZ;
% Vpacking = maxHeight * container.sizeX * container.sizeY;
Vpacking = allModelsBB.volume;
if isFeasible
    fitness = Vpacking;
else
    A = overallBB.volume / container.volume;
    Voverlap = sum(overlapVolume,'all');
    fitness = wA * (A - 1) + wO * Voverlap + Vpacking;
end

end