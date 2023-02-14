function [fitness,isFeasible] = fitnessFunction(models,container)

%% check if the current status is a feasible solution
isFeasible = true;

% check if all objects are inside the container
overallBoundingBox = models(1).BoundingBox;
for i = 2:length(models)
    overallBoundingBox = overallBoundingBox.add(models(i).BoundingBox);
end
if ~overallBoundingBox.isInside(container)
    isFeasible = false;
end

% for iModel = models
%     if ~iModel.BoundingBox.isInside(container)
%         isFeasible = false;
%         break
%     end
% end

% check if objects collide with each other
overlapVolume = zeros(length(models));
if isFeasible
    for i = 1:length(models)
        for j = [1:(i-1) (i+1):length(models)]
            if detectCollision(models(i),models(j),container.minDist)
                isFeasible = false;
                [~,overlapVolume(i,j)] = checkOverlap(models(i).BoundingBox,models(j).BoundingBox);
            end
        end
    end
end

%% calculate the fitness function
wA = 100;
wO = 1;

maxHeight = overallBoundingBox.maxZ;
Vpacking = maxHeight * container.sizeX * container.sizeY;
if isFeasible
    fitness = Vpacking;
else
    A = overallBoundingBox.volume / container.volume;
    Voverlap = sum(overlapVolume,'all');
    fitness = wA * (A - 1) + wO * Voverlap + container.volume;
end

end