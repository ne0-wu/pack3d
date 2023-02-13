function fitness = fitnessFunction(models,container)

%% check if the current status is a feasible solution
isFeasible = true;

% check if all objects are inside the container
for iModel = models
    if ~iModel.isInside(container)
        isFeasible = false;
        break
    end
end

% check if objects collide with each other
if isFeasible
    for iModel = models
        for jModel = models
            if isequal(iModel,jModel), continue, end
            if detectCollision(iModel,jModel)
                isFeasible = false;
                break;
            end
        end
        if ~isFeasible
            break;
        end
    end
end

isFeasible

end