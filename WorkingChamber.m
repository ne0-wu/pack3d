classdef WorkingChamber < handle

    properties
        containerSize % size of the working chamber
        minDist % minimum distance between models allowed
        models
        overlapVolume = zeros(100);
    end

    methods

        function obj = WorkingChamber(length, width, height, dist)
            obj.containerSize = [length width height];
            if nargin == 4
                obj.minDist = dist;
            else
                obj.minDist = 0;
            end
        end

        function output = containerVolume(obj)
            output = prod(obj.containerSize);
        end

        function appendModel(obj, models)
            % append a model into the working chamber
            numModelsBef = length(obj.models);
            if isempty(obj.models)
                obj.models = models;
            else
                obj.models = [obj.models, models];
            end
            numModelsAft = length(obj.models);
            obj.update((numModelsBef + 1):numModelsAft);
        end

        function update(obj, iModified)
            % update the collision status of models
            numModels = length(obj.models);
            if numModels > size(obj.overlapVolume, 1)
                obj.overlapVolume(numModels, numModels) = 0;    % expand the matrix
            end
            if nargin == 1
                iModified = 1:numModels;
            end
            obj.overlapVolume(iModified, :) = 0;
            obj.overlapVolume(:, iModified) = 0;
            for i = iModified
                for j = [1:i-1 i+1:numModels]
                    if checkModelCollision(obj.models(i), obj.models(j), obj.minDist)
                        [~, obj.overlapVolume(i, j)] = ...
                            checkAabbCollision(obj.models(i).BoundingBox, ...
                                               obj.models(j).BoundingBox);
                        obj.overlapVolume(j, i) = obj.overlapVolume(i, j);
                    end
                end
            end
        end

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
%             Vpacking = allModelsBB.volume;
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

        function draw(obj, fig)
            if nargin == 1
                fig = figure;
            end
            for iModified = 1:length(obj.models)
                obj.models(iModified).draw(fig);
                hold on
            end
            drawnow
            ax = zeros([1, 6]);
            ax([2 4 6]) = obj.containerSize;
            axis(ax);
            hold off
        end

    end

end
