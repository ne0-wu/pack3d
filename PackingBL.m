classdef PackingBL < Packing

    methods

        % just excute this function
        function pack(obj)
            obj.bottomLeft();
        end

        % make the model fall into a specific direction
        % (to fall means to move and stop until collision happens)
        function error = fall(obj, iModel, direction, maxDistance, fig)
            for i = iModel
                % check if it's ok to do the fall
                if sum(obj.overlapVolume(i, :), 'all') > 0
                    disp('cant perform the fall')
                    error = true;
                    return
                end

                initPos = obj.models(i).position();
                direction = direction / norm(direction);
                distNear = 0; distFar = maxDistance;
                while abs(distFar - distNear) > obj.minDist / 3
                    distMid = (distNear + distFar) / 2;

                    obj.models(i).moveTo(initPos + direction * distMid);
                    obj.update(i);

                    bNoCollision = sum(obj.overlapVolume(i, :), 'all') == 0;
                    bIsInside = obj.insideContainer(i);
                    if bNoCollision && bIsInside                        
                        distNear = distMid;
                        distBest = distMid;
                    else
                        distFar = distMid;
                        distBest = distNear;
                    end

                    if nargin == 5
                         obj.draw(fig);
                    end
                end
                obj.models(i).moveTo(initPos + direction * distBest);
                obj.update(i);
            end
            error = false;
        end

        % bottom-left method
        function error = bottomLeft(obj, packingSequence, orientation, fig)
            % default packing sequence
            if nargin < 2
                packingSequence = 1:obj.numModels();
            end
            % set orientation of models
            if nargin >= 3
                obj.setOrientation(orientation);
            end

            % initilize: move all models to somewhere very far away
            for i = 1:obj.numModels()
                obj.models(i).moveTo(obj.containerSize * 100);
            end

            % bottom-left
            for i = packingSequence
                upperRightFarPoint = obj.models(i).BoundingBox.box([2 4 6]);
                initPos = obj.models(i).position() + obj.containerSize - upperRightFarPoint ...
                    - obj.minDist / 3;

                obj.models(i).moveTo(initPos);
                obj.update(i);

                if nargin < 4
                    % fall in z axis
                    error = obj.fall(i, [0 0 -1], obj.models(i).BoundingBox.minZ);
                    if error, return; end

                    % fall in y axis
                    error = obj.fall(i, [0 -1 0], obj.models(i).BoundingBox.minY);
                    if error, return; end

                    % fall in x axis
                    error = obj.fall(i, [-1 0 0], obj.models(i).BoundingBox.minX);
                    if error, return; end
                else
                    % fall in z axis
                    error = obj.fall(i, [0 0 -1], obj.models(i).BoundingBox.minZ, fig);
                    if error, return; end

                    % fall in y axis
                    error = obj.fall(i, [0 -1 0], obj.models(i).BoundingBox.minY, fig);
                    if error, return; end

                    % fall in x axis
                    error = obj.fall(i, [-1 0 0], obj.models(i).BoundingBox.minX, fig);
                    if error, return; end
                end
            end
        end

        % fitness function
        % (total packing volume of the reuslt of BL method using given packing sequence and orientation)
        function fitness = fitnessFunc(obj, packingSequence, orientation, fig)
            obj.bottomLeft(packingSequence, orientation);
            fitness = obj.packingVolume();
        end

    end

end