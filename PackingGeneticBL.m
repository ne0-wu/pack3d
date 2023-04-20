classdef PackingGeneticBL < PackingBL

    properties
        % each line of population is [packing sequence, orientation of models]
        population

        % data to be stored
        indexSurvived
        fitnessList

        % history
        history

        % figure
        fig
    end
    
    methods

        function pack(obj, maximumSteps, populationSize)
            if nargin < 2
                maximumSteps = 100;
            end
            if nargin < 3
                populationSize = 20;
            end
            obj.geneticAlgorithm(maximumSteps, populationSize);
        end

        %% utils
        function sz = populationSize(obj)
            sz = size(obj.population, 1);
        end

        function sequence = packingSequence(obj, i)
            sequence = obj.population(i, 1:obj.numModels(), 1);
        end

        function orient = orientation(obj, i)
            orient = obj.population(i, 1:obj.numModels(), [2 3 4]);
        end

        %% initialize the population
        function initialize(obj, populationSize)
            obj.population = PackingGeneticBL.generatePopulation(obj.numModels(), obj.k, populationSize);
        end

        %% select best part of the population to survive
        function indexSurvived = naturalSelection(obj, numSurvived)
            if nargin < 2
                numSurvived = floor(obj.populationSize() / 2);
            end
            
            % compute fitness
            for i = setdiff(1:obj.populationSize(), obj.indexSurvived)
                obj.fitnessList(i) = obj.fitnessFunc(obj.packingSequence(i), obj.orientation(i));
            end

            % select k chromosomes with min fitness
            [~, indexSurvived] = mink(obj.fitnessList, numSurvived);
        end

        %% solve the optimization problem with genetic algorithm
        function geneticAlgorithm(obj, maximumSteps, populationSize)
            % initialize
            obj.initialize(populationSize);
            numSurvived = floor(obj.populationSize() / 3);
            numChildren = floor(obj.populationSize() / 3);
            numGenerated = obj.populationSize() - numSurvived - numChildren;

            obj.fitnessList = zeros(obj.populationSize(), 1);

            obj.history = zeros(maximumSteps, 1);
            obj.fig = figure;

            stepCount = 0;
            while stepCount < maximumSteps
                tic

                stepCount = stepCount + 1;
                
                % natural selection
                obj.indexSurvived = obj.naturalSelection(numSurvived);
                indexDied = setdiff(1:populationSize, obj.indexSurvived);

                % breed
                obj.population(indexDied(1:numChildren), :, :) = ...
                    PackingGeneticBL.breed(obj.population(obj.indexSurvived, :, :), numChildren);

                % generate
                obj.population(indexDied(numChildren + (1:numGenerated)), :, :) = ...
                    PackingGeneticBL.generatePopulation(obj.numModels(), obj.k, numGenerated);

                obj.history(stepCount) = obj.fitnessList(obj.indexSurvived(1));

                % disp
                disp({stepCount, obj.fitnessList(obj.indexSurvived(1))})
                toc

                % draw current best
                indexBest = obj.indexSurvived(1);
                obj.bottomLeft(obj.packingSequence(indexBest), obj.orientation(indexBest));
                obj.draw(obj.fig);

                zlim([0 5])
                drawnow
            end
        end

    end

    methods(Static)

        %% generate a population
        function population = generatePopulation(numModels, k, populationSize)
            population = zeros(populationSize, numModels, 4);
            for i = 1:populationSize
                % packing sequence
                population(i, 1:numModels, 1) = randperm(numModels);
                % orientation
                population(i, 1:numModels, [2 3 4]) = randi([0 k], [1 numModels 3]);
            end
        end

        %% breed
        function children = breed(parents, numChildren)
            if nargin < 2
                numChildren = 1;
            end
            
            numParents = size(parents, 1); % num of chromosomes as parents
            numModels = size(parents, 2);  % num of models
            children = zeros(numChildren, numModels, 4);
            for i = 1:numChildren
                indexFather = randi(numParents);
                indexMother = randi(numParents);
                children(i, :, :) = PackingGeneticBL.crossover( ...
                    parents(indexFather, :, :), parents(indexMother, :, :), rand());
            end
        end

        %% crossover
        function child = crossover(father, mother, p)
            if nargin < 3
                p = 0.5;
            end

            numModels = size(father, 2);
            child = zeros(1, numModels, 4);

            % crossover of the packing sequence
            tempListFather = zeros(numModels, 1);
            tempListFather(father(:, :, 1)) = 1:numModels;
            tempListMother = zeros(numModels, 1);
            tempListMother(mother(:, :, 1)) = 1:numModels;
            tempListChild = tempListFather * p + tempListMother * (1 - p) ... % crossover
                + rand(numModels, 1) * numModels / 4; % mutation
            [~, sequenceChild] = sort(tempListChild);

            % crossover of orientation
            randomArray = rand(1, numModels, 3);
            orientChild = (randomArray < p) .* father(:, :, [2 3 4]) + ...
                          (randomArray >= p) .* mother(:, :, [2 3 4]);

            % return
            child(1, :, 1) = sequenceChild;
            child(1, :, [2 3 4]) = orientChild;
        end

    end

end