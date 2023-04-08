function models = simulatedAnnealing(models, container, maximumSteps)
    fig = figure;

    if nargin == 2
        maximumSteps = 1e5;
    end

    tmpt = 200; % initial temperature
    tmptEnd = 0.1; % end temperature
    alpha = 0.999; % cooling rate

    probabilityMove = 0.8;
    rotAng = 90;

    fOld = fitnessFunction(models, container);

    stepCount = 0;

    while tmpt > tmptEnd && stepCount < maximumSteps
        stepCount = stepCount + 1;
        %% generate a new solution
        i = randi(length(models)); % the object to make a small change
        modelTmp = copy(models(i)); % backup this object in case of it need to be recovered
        modelTmp.Color = models(i).Color;

        if rand < probabilityMove
            % move the object with some probability
            deltaR = [0 0 0];
            deltaR(randi(3)) = randn * 0.5;
            models(i).move(deltaR);
        else
            % otherwise rotate the object
            models(i).rotate(rotAng * randi(floor(360 / rotAng)), randi(3));
        end

        %% calculate the fitness function
        [fNew, isFeasible] = fitnessFunction(models, container);

        %% determine to accept it or not
        if rand < acceptProbability(fNew - fOld, tmpt)
            % accept
            fOld = fNew;
            tmpt = tmpt * alpha;
            disp({stepCount fNew isFeasible})

            for i = 1:length(models)
                models(i).draw(fig);
                hold on
            end

            drawnow
            hold off
        else
            % reject
            models(i) = modelTmp;
        end

    end

end

function p = acceptProbability(deltaF, tmpt)

    if deltaF <= 0
        p = 1;
    else
        p = exp(-deltaF / tmpt);
        disp({deltaF p})
    end

end
