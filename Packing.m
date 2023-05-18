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

        % rotation is limited to multiples of 2pi/k
        k
    end

    methods

        function obj = Packing(length, width, height, dist, kk)
            obj.containerSize = [length width height];
            if nargin < 4
                obj.minDist = 0.1;
            else
                obj.minDist = dist;
            end
            if nargin < 5
                obj.k = 4;
            else
                obj.k = kk;
            end
        end

        %% append some models
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

        %% utils
        % num of models
        function num = numModels(obj)
            num = length(obj.models);
        end

        % volume of the container
        function volume = containerVolume(obj)
            volume = prod(obj.containerSize);
        end

        % max height of all models
        function height = maxHeight(obj)
            height = obj.models(1).BoundingBox.maxZ();
            for i = 2:length(obj.models)
                height = max(height, obj.models(i).BoundingBox.maxZ());
            end
        end

        % packing volume
        function volume = packingVolume(obj)
            volume = obj.maxHeight() * obj.containerSize(1) * obj.containerSize(2);
        end

        %% collision detection
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
        
        %% set orientation of models (for bottom-left)
        function setOrientation(obj, orientation)
            for i = 1:obj.numModels()
                rot = rotx(orientation(1, i, 1) * 360 / obj.k) ...
                    * roty(orientation(1, i, 2) * 360 / obj.k) ...
                    * rotz(orientation(1, i, 3) * 360 / obj.k);
                obj.models(i).setOrientation(rot);
            end
        end

        %% draw
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
            ax(6) = min(obj.maxHeight, ax(6));
            axis(ax);

            light

            hold off
            drawnow
        end

    end

end
