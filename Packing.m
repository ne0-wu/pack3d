classdef Packing < handle

    properties
        % size of the container
        containerSize

        % minimum distance between models allowed
        minDist

        % list of models
        models

        % intermediate variables to be saved
        overlapVolume = zeros(100);
        insideContainer = zeros(100, 1);
    end

    methods

        function obj = Packing(length, width, height, dist)
            obj.containerSize = [length width height];
            if nargin == 3
                obj.minDist = 0.1;
            else
                obj.minDist = max(dist, 0.1);
            end
        end

        % volume of the container
        function output = containerVolume(obj)
            output = prod(obj.containerSize);
        end

        % append some models
        function appendModel(obj, models)
            numModelsBef = length(obj.models);
            if isempty(obj.models)
                obj.models = models;
            else
                obj.models = [obj.models, models];
            end
            numModelsAft = length(obj.models);
            obj.update((numModelsBef + 1):numModelsAft);
        end

        function height = maxHeight(obj)
            height = obj.models(1).BoundingBox.maxZ();
            for i = 2:length(obj.models)
                height = max(height, obj.models(i).BoundingBox.maxZ());
            end
        end

        % update the collision status of models
        function collisionStatus = checkCollisionIJ(obj, i, j)
            if i == j
                collisionStatus = false;
            else
                collisionStatus = checkModelCollision( ...
                    obj.models(i), obj.models(j), obj.minDist);
            end
        end

        % overlap volume of aabbs
        function volume = aabbOverlapVolume(obj, i, j)
            if i == j
                volume = 0;
            else
                [~, volume] = checkAabbOverlap( ...
                    obj.models(i).BoundingBox, obj.models(j).BoundingBox);
            end
        end

        % update the collision status of models
        function update(obj, iModified)
            numModels = length(obj.models);
            if nargin == 1
                iModified = 1:numModels;
            end

            % expand the matrix if necessary
            if numModels > size(obj.overlapVolume, 1)
                obj.overlapVolume(numModels, numModels) = 0;    
            end

            % check if models are inside the container
            for i = 1:numModels
                obj.insideContainer(i) = ...
                    obj.models(i).BoundingBox.isInside(obj.containerSize);
            end

            % check if models overlap with each other
            obj.overlapVolume(iModified, :) = 0;
            obj.overlapVolume(:, iModified) = 0;
            for i = iModified
                for j = 1:numModels
                    if obj.insideContainer(i) && obj.insideContainer(j)
                        if obj.checkCollisionIJ(i, j)
                            obj.overlapVolume(i, j) = obj.aabbOverlapVolume(i, j);
                            obj.overlapVolume(j, i) = obj.overlapVolume(i, j);
                        end
                    end 
                end
            end
        end
        
        % draw all the models
        function draw(obj, fig)
            if nargin == 1
                fig = figure;
            end

            for i = 1:length(obj.models)
                obj.models(i).draw(fig);
                hold on
            end

            % set the axes
            ax = zeros([1, 6]);
            ax([2 4 6]) = obj.containerSize;
            axis(ax);

            hold off
            drawnow
        end

    end

end