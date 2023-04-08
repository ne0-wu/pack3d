classdef PackingSA < Packing

    methods

        % just excute this function
        function pack(obj, maximumSteps)
            obj.simulatedAnnealing(maximumSteps);
        end

        % fitness function to be optimized
        function [fitness, isFeasible] = fitnessFunc(obj, iModified)
            if nargin == 2
                obj.update(iModified);
            else
                % obj.update();
            end

            %% check if the current status is a feasible solution
            isFeasible = true;
            % check if objects collide with each other
            if sum(obj.overlapVolume, "all") > 0
                isFeasible = false;
            end
            % check if all objects are inside the container
            allModelsBB = obj.models(1).BoundingBox;
            for i = 2:length(obj.models)
                allModelsBB = allModelsBB.aabbUnion(obj.models(i).BoundingBox);
            end
            if ~allModelsBB.isInside(obj.containerSize)
                isFeasible = false;
            end

            %% calculate the fitness function
            wA = 1000; wO = 100;
            maxHeight = allModelsBB.maxZ;
            Vpacking = maxHeight * obj.containerSize(1) * obj.containerSize(2);
            % Vpacking = allModelsBB.volume;
            if isFeasible
                fitness = Vpacking;
            else
                overallBB = AABB();
                overallBB.box([2 4 6]) = obj.containerSize;
                overallBB = overallBB.aabbUnion(allModelsBB);
                A = overallBB.volume / obj.containerVolume;
                Voverlap = sum(obj.overlapVolume,'all');
                fitness = wA * (A - 1) + wO * Voverlap + Vpacking;
            end
        end

        % optimize with the simulated annealing algorithm
        function simulatedAnnealing(obj, maximumSteps)
            if nargin == 1
                maximumSteps = 1e4;
            end

            fig = figure;

            tmpt = 200; % initial temperature
            tmptEnd = 0.1; % end temperature
            alpha = 0.999; % cooling rate
        
            probabilityMove = 0.8;
            rotAng = 90;
        
            fOld = obj.fitnessFunc();
        
            stepCount = 0;
            while tmpt > tmptEnd && stepCount < maximumSteps
                stepCount = stepCount + 1;
                %% generate a new solution
                iModified = randi(length(obj.models)); % the object to make a small change
                modelTmp = copy(obj.models(iModified)); % backup this object in case of it need to be recovered
                ovTmp = obj.overlapVolume;
                modelTmp.Color = obj.models(iModified).Color;
        
                if rand < probabilityMove
                    % move the object by some probability
                    posDiff = [0 0 0];
                    posDiff(randi(3)) = randn * 0.5;
                    obj.models(iModified).move(posDiff);
                else
                    % otherwise rotate the object
                    obj.models(iModified).rotate(rotAng * randi(floor(360 / rotAng)), randi(3));
                end
        
                %% calculate the fitness function
                [fNew, isFeasible] = obj.fitnessFunc(iModified);
        
                %% determine to accept it or not
                acceptProbability = @(deltaF, tmpt) (deltaF <= 0) + (deltaF > 0) * exp(-deltaF / tmpt);

                if rand < acceptProbability(fNew - fOld, tmpt)
                    % accept
                    fOld = fNew;
                    tmpt = tmpt * alpha;
                    disp({stepCount fNew isFeasible})
                    obj.draw(fig);
                else
                    % reject
                    obj.models(iModified) = modelTmp;
                    obj.overlapVolume = ovTmp;
                end
            end
        end

    end

end