classdef PackingBL < Packing

    properties
        isPacked
    end

    methods

        % just excute this function
        function pack(obj, maximumSteps)
            obj.init();
            obj.bottomLeft();
        end

        % move all models to somewhere very far away
        function init(obj)
            for i = 1:length(obj.models)
                obj.models(i).moveTo(obj.containerSize * 100);
            end
        end

        % make the model fall in a specific direction
        % to fall means to move and stop until collision happens
        function fall(obj, iModel, direction, maxDistance)
            for i = iModel
                % check if it's ok to do the fall
                if sum(obj.overlapVolume(i, :), 'all') > 0
                    disp('fall error')
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
        end

        function bottomLeft(obj, packingOrder)
            if nargin == 1
                packingOrder = 1:length(obj.models);
            end

            for i = packingOrder
                upperRightFarPoint = obj.models(i).BoundingBox.box([2 4 6]);
                initPos = obj.models(i).position() + obj.containerSize - upperRightFarPoint ...
                    - obj.minDist / 3;
                obj.models(i).moveTo(initPos);
                obj.update(i);
                obj.fall(i, [0 0 -1], obj.models(i).BoundingBox.minZ);
                obj.fall(i, [0 -1 0], obj.models(i).BoundingBox.minY);
                obj.fall(i, [-1 0 0], obj.models(i).BoundingBox.minX);
            end
        end

    end

end