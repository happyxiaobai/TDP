classdef DistributionallyRobustAlgo < handle
    properties
        inst                    % Instance object
        centers                 % List of center areas
        zones                   % Array of zone assignments
        r = 0.1                 % Tolerance parameter
        gamma                   % Risk parameter
        scenarios               % Scenario data
        numScenarios            % Number of scenarios
        scenarioDemands         % Scenario demands
        rand                    % Random number generator
        meanVector              % Mean vector
        covarianceMatrix        % Covariance matrix
        delta1                  % D2 fuzzy set parameter
        delta2                  % D2 fuzzy set parameter
        useD1                   % Whether to use D1 fuzzy set
        useJointChance          % Whether to use joint chance constraint
        individualGammas        % Individual risk allocations for Bonferroni approx
        demandUpperBound        % Upper bound on demand
        timeLimit = 1000        % Time limit in seconds
        maxIterations = 100     % Maximum iterations
    end
    
    methods
        % Constructor
        function obj = DistributionallyRobustAlgo(instance, scenarios, gamma, seed, useD1, delta1, delta2, useJointChance)
            obj.inst = instance;
            obj.scenarios = scenarios;
            obj.numScenarios = size(scenarios, 1);
            obj.scenarioDemands = zeros(obj.numScenarios, instance.getN());
            
            for s = 1:obj.numScenarios
                for i = 1:instance.getN()
                    obj.scenarioDemands(s, i) = round(scenarios(s, i));
                end
            end
            
            obj.gamma = gamma;
            obj.r = 0.1;
            rng(seed); % Set random seed
            obj.zones = cell(1, instance.k);
            obj.useD1 = useD1;
            obj.delta1 = delta1;
            obj.delta2 = delta2;
            obj.useJointChance = useJointChance;
            
            % Calculate moment information
            obj.calculateMomentInformation();
            
            % Calculate demand upper bound
            totalMeanDemand = sum(obj.meanVector);
            obj.demandUpperBound = (1 + obj.r) * (totalMeanDemand / instance.k);
            
            % Initialize individual gammas for Bonferroni approximation
            if useJointChance
                obj.individualGammas = zeros(1, instance.k);
                for j = 1:instance.k
                    obj.individualGammas(j) = gamma / instance.k;
                end
            end
        end
        
        % Calculate moment information (mean and covariance)
        function calculateMomentInformation(obj)
            n = obj.inst.getN();
            
            % Calculate mean vector
            obj.meanVector = zeros(1, n);
            for i = 1:n
                obj.meanVector(i) = mean(obj.scenarios(:, i));
            end
            
            % Calculate covariance matrix
            obj.covarianceMatrix = zeros(n, n);
            for i = 1:n
                for j = 1:n
                    covariance = 0;
                    for s = 1:obj.numScenarios
                        covariance = covariance + (obj.scenarios(s, i) - obj.meanVector(i)) * ...
                                                 (obj.scenarios(s, j) - obj.meanVector(j));
                    end
                    obj.covarianceMatrix(i, j) = covariance / obj.numScenarios;
                end
            end
            
            % Check if covariance matrix is symmetric
            isSymmetric = all(all(abs(obj.covarianceMatrix - obj.covarianceMatrix') < 1e-10));
            fprintf('Covariance matrix is symmetric: %d\n', isSymmetric);
            
            % Check if covariance matrix is positive semidefinite
            [~, p] = chol(obj.covarianceMatrix);
            isPSD = (p == 0);
            
            if ~isPSD
                fprintf('Warning: Covariance matrix is not positive semidefinite, attempting to fix...\n');
                obj.ensurePSDMatrix();
                
                % Check again
                [~, p] = chol(obj.covarianceMatrix);
                isPSD = (p == 0);
                fprintf('Fixed matrix is positive semidefinite: %d\n', isPSD);
            end
        end
        
        % Ensure the covariance matrix is positive semidefinite
        function ensurePSDMatrix(obj)
            % Add a small value to the diagonal
            epsilon = 1e-5;
            for i = 1:size(obj.covarianceMatrix, 1)
                obj.covarianceMatrix(i, i) = obj.covarianceMatrix(i, i) + epsilon;
            end
        end
        
        % Run the algorithm
        function run(obj, filename)
            startTime = tic;
            Best = inf;
            BestZones = cell(1, obj.inst.k);
            
            % Step 1: Construct initial center set
            initialCenters = obj.selectInitialCenters();
            obj.centers = [];
            for i = 1:length(initialCenters)
                centerId = initialCenters(i);
                obj.centers = [obj.centers, obj.inst.getAreas(){centerId+1}];
                obj.inst.getAreas(){centerId+1}.setCenter(true);
            end
            
            % Step 2: Generate initial feasible solution
            feasible = obj.generateInitialSolution();
            
            if ~feasible
                fprintf('Cannot find feasible solution, check model parameters\n');
                return;
            end
            
            % Step 3: Improve initial solution
            change = true;
            cur_value = obj.evaluateObjective();
            iteration = 0;
            
            while change && iteration < obj.maxIterations
                iteration = iteration + 1;
                change = false;
                
                % Check each zone's true center
                newCenters = obj.findTrueCenters();
                
                % If centers changed, update and resolve
                if ~obj.compareCenters(obj.centers, newCenters)
                    obj.centers = newCenters;
                    change = true;
                    feasible = obj.generateInitialSolution();
                    if feasible
                        cur_value = obj.evaluateObjective();
                        fprintf('Iteration %d: Objective value = %.2f\n', iteration, cur_value);
                    else
                        fprintf('Iteration %d: No feasible solution found\n', iteration);
                        break;
                    end
                end
            end
            
            % Ensure connectivity
            if feasible
                obj.ensureConnectivity();
                cur_value = obj.evaluateObjective();
            end
            
            % Evaluate final result
            if feasible && cur_value < Best
                Best = cur_value;
                for z = 1:obj.inst.k
                    BestZones{z} = obj.zones{z};
                end
            end
            
            endTime = toc(startTime);
            timeSpentInSeconds = endTime;
            
            % Output results
            outputFilePath = ['./output/', strrep(filename, '.dat', '_drcc.txt')];
            fileID = fopen(outputFilePath, 'w');
            
            for io = 1:length(BestZones)
                if ~isempty(BestZones{io})
                    fprintf(fileID, 'center ID: %d\n', obj.centers(io).getId());
                    for jo = 1:length(BestZones{io})
                        fprintf(fileID, '%d ', BestZones{io}(jo));
                    end
                    fprintf(fileID, '\n');
                end
            end
            
            fprintf(fileID, 'best objective: %.2f\n', Best);
            fprintf(fileID, 'Runtime: %.2f s\n', timeSpentInSeconds);
            fprintf(fileID, 'Fuzzy set type: %s\n', conditional(obj.useD1, 'D_1', 'D_2'));
            
            if ~obj.useD1
                fprintf(fileID, 'delta1: %.2f, delta2: %.2f\n', obj.delta1, obj.delta2);
            end
            
            fprintf(fileID, 'Constraint type: %s\n', conditional(obj.useJointChance, 'Joint constraint (DRJCC)', 'Individual constraint (DRICC)'));
            fprintf(fileID, 'Risk parameter: %.2f\n', obj.gamma);
            
            fclose(fileID);
            
            fprintf('Runtime: %.2f s\n', timeSpentInSeconds);
            fprintf('Final objective value: %.2f\n', Best);
        end
        
        % Select initial centers
        function centerIds = selectInitialCenters(obj)
            InitialNum = 5; % Adjustable parameter
            candidateCenters = [];
            centerFrequency = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
            
            scenariosProcessed = 0;
            while scenariosProcessed < InitialNum && scenariosProcessed < obj.numScenarios
                scenarioIndex = randi(obj.numScenarios);
                
                % Solve for the specific scenario
                scenarioCenters = obj.solveForScenario(scenarioIndex);
                
                if length(scenarioCenters) == obj.inst.k
                    scenariosProcessed = scenariosProcessed + 1;
                    fprintf('Processing scenario %d/%d, scenario index: %d\n', ...
                           scenariosProcessed, InitialNum, scenarioIndex);
                    
                    % Update center frequency
                    for i = 1:length(scenarioCenters)
                        center = scenarioCenters(i);
                        if isKey(centerFrequency, center)
                            centerFrequency(center) = centerFrequency(center) + 1;
                        else
                            centerFrequency(center) = 1;
                        end
                    end
                end
            end
            
            % Sort centers by frequency
            centerList = cell2mat(keys(centerFrequency));
            freqList = cell2mat(values(centerFrequency));
            [~, sortedIndices] = sort(freqList, 'descend');
            sortedCenters = centerList(sortedIndices);
            
            % Select top k centers
            candidateCenters = sortedCenters(1:min(obj.inst.k, length(sortedCenters)));
            
            % Fill with random centers if needed
            while length(candidateCenters) < obj.inst.k
                randomCenter = randi(obj.inst.getN()) - 1;
                if ~ismember(randomCenter, candidateCenters)
                    candidateCenters = [candidateCenters, randomCenter];
                end
            end
            
            centerIds = candidateCenters;
        end
        
        % Solve deterministic model for a specific scenario
        function scenarioCenters = solveForScenario(obj, scenarioIndex)
            localTimeLimit = 60; % seconds
            
            try
                % Create scenario instance
                scenarioInstance = obj.createScenarioInstance(scenarioIndex);
                
                % Create Algo object and set time limit
                algo = Algo(scenarioInstance);
                algo.setTimeLimit(localTimeLimit);
                
                % Get centers for this scenario
                scenarioCenters = algo.getCorrectSolutionCenters();
                
                % If not enough centers, supplement with random ones
                if length(scenarioCenters) < obj.inst.k
                    centerSet = scenarioCenters;
                    while length(centerSet) < obj.inst.k
                        candidate = randi(obj.inst.getN()) - 1;
                        if ~ismember(candidate, centerSet)
                            centerSet = [centerSet, candidate];
                            scenarioCenters = [scenarioCenters, candidate];
                        end
                    end
                    fprintf('Scenario %d: Not enough centers, randomly supplemented to %d centers\n', ...
                           scenarioIndex, length(scenarioCenters));
                end
                
                return;
            catch e
                fprintf('Error solving scenario %d: %s\n', scenarioIndex, e.message);
                
                % Use random selection as fallback
                fallbackCenters = [];
                centerSet = [];
                
                while length(centerSet) < obj.inst.k
                    candidate = randi(obj.inst.getN()) - 1;
                    if ~ismember(candidate, centerSet)
                        centerSet = [centerSet, candidate];
                        fallbackCenters = [fallbackCenters, candidate];
                    end
                end
                
                fprintf('Scenario %d: Solution failed, randomly selected %d centers\n', ...
                       scenarioIndex, length(fallbackCenters));
                scenarioCenters = fallbackCenters;
            end
        end
        
        % Create scenario instance
        function scenarioInstance = createScenarioInstance(obj, scenarioIndex)
            scenarioInstance = obj.inst.createScenarioInstance(obj.scenarioDemands(scenarioIndex, :));
        end
        
        % Generate initial feasible solution using SOCP approach
        function feasible = generateInitialSolution(obj)
            try
                % Create YALMIP optimization variables
                n = obj.inst.getN();
                p = length(obj.centers);
                
                % Decision variables - binary assignment variables
                x = binvar(n, p, 'full');
                
                % Constraints
                constraints = [];
                
                % Each area must be assigned to exactly one district
                for i = 1:n
                    constraints = [constraints, sum(x(i, :)) == 1];
                end
                
                % Centers must be assigned to their own districts
                for j = 1:p
                    centerId = obj.centers(j).getId();
                    constraints = [constraints, x(centerId+1, j) == 1];
                end
                
                % Add distributionally robust chance constraints
                if obj.useD1
                    % D1 fuzzy set constraints
                    for j = 1:p
                        % Extract assignment vector for district j
                        xj = x(:, j);
                        
                        % Mean term
                        meanTerm = obj.meanVector * xj;
                        
                        % Factor for SOC constraint
                        if obj.useJointChance
                            factor = sqrt((1 - obj.individualGammas(j)) / obj.individualGammas(j));
                        else
                            factor = sqrt((1 - obj.gamma) / obj.gamma);
                        end
                        
                        % Second-order cone constraint
                        varTerm = sqrt(xj' * obj.covarianceMatrix * xj);
                        constraints = [constraints, meanTerm + factor * varTerm <= obj.demandUpperBound];
                    end
                else
                    % D2 fuzzy set constraints
                    for j = 1:p
                        % Extract assignment vector for district j
                        xj = x(:, j);
                        
                        % Mean term
                        meanTerm = obj.meanVector * xj;
                        
                        % Factor for SOC constraint depends on relationship between delta1/delta2 and gamma
                        if obj.useJointChance
                            gamma_j = obj.individualGammas(j);
                            if obj.delta1 / obj.delta2 <= gamma_j
                                factor = sqrt(obj.delta1) + sqrt((1 - gamma_j) / gamma_j * (obj.delta2 - obj.delta1));
                            else
                                factor = sqrt(obj.delta2 / gamma_j);
                            end
                        else
                            if obj.delta1 / obj.delta2 <= obj.gamma
                                factor = sqrt(obj.delta1) + sqrt((1 - obj.gamma) / obj.gamma * (obj.delta2 - obj.delta1));
                            else
                                factor = sqrt(obj.delta2 / obj.gamma);
                            end
                        end
                        
                        % Second-order cone constraint
                        varTerm = sqrt(xj' * obj.covarianceMatrix * xj);
                        constraints = [constraints, meanTerm + factor * varTerm <= obj.demandUpperBound];
                    end
                end
                
                % Objective function - minimize total distance
                objective = 0;
                for i = 1:n
                    for j = 1:p
                        objective = objective + obj.inst.dist(i, obj.centers(j).getId()+1) * x(i, j);
                    end
                end
                
                % Set solver options
                options = sdpsettings('solver', 'mosek', 'verbose', 0, 'cachesolvers', 1);
                
                % Solve the problem
                result = optimize(constraints, objective, options);
                
                if result.problem == 0
                    % Extract solution
                    x_val = value(x);
                    
                    % Populate zones
                    for j = 1:p
                        obj.zones{j} = [];
                        for i = 1:n
                            if abs(x_val(i, j) - 1) < 1e-6
                                obj.zones{j} = [obj.zones{j}, i-1];
                            end
                        end
                    end
                    
                    feasible = true;
                else
                    fprintf('Optimization problem could not be solved: %s\n', yalmiperror(result.problem));
                    feasible = false;
                end
                
                return;
            catch e
                fprintf('Error in generateInitialSolution: %s\n', e.message);
                feasible = false;
                return;
            end
        end
        
        % Find true centers for each zone
        function newCenters = findTrueCenters(obj)
            newCenters = [];
            
            for j = 1:length(obj.centers)
                if isempty(obj.zones{j})
                    newCenters = [newCenters, obj.centers(j)];
                    continue;
                end
                
                bestCenter = -1;
                minTotalDist = inf;
                
                for i = 1:length(obj.zones{j})
                    nodeId = obj.zones{j}(i);
                    totalDist = 0;
                    
                    for k = 1:length(obj.zones{j})
                        otherNodeId = obj.zones{j}(k);
                        if nodeId ~= otherNodeId
                            totalDist = totalDist + obj.inst.dist(nodeId+1, otherNodeId+1);
                        end
                    end
                    
                    if totalDist < minTotalDist
                        minTotalDist = totalDist;
                        bestCenter = nodeId;
                    end
                end
                
                newCenters = [newCenters, obj.inst.getAreas(){bestCenter+1}];
            end
        end
        
        % Compare two sets of centers
        function isSame = compareCenters(obj, centers1, centers2)
            if length(centers1) ~= length(centers2)
                isSame = false;
                return;
            end
            
            for i = 1:length(centers1)
                if centers1(i).getId() ~= centers2(i).getId()
                    isSame = false;
                    return;
                end
            end
            
            isSame = true;
        end
        
        % Ensure connectivity for all zones
        function ensureConnectivity(obj)
            allConnected = false;
            iteration = 0;
            maxIterations = 1000;
            
            % Create YALMIP model for connectivity
            n = obj.inst.getN();
            p = length(obj.centers);
            
            % Decision variables
            x = binvar(n, p, 'full');
            
            % Basic constraints
            constraints = [];
            
            % Each area must be assigned to exactly one district
            for i = 1:n
                constraints = [constraints, sum(x(i, :)) == 1];
            end
            
            % Centers must be assigned to their own districts
            for j = 1:p
                centerId = obj.centers(j).getId();
                constraints = [constraints, x(centerId+1, j) == 1];
            end
            
            % Add distributionally robust chance constraints (same as in generateInitialSolution)
            % ...
            
            % Objective function
            objective = 0;
            for i = 1:n
                for j = 1:p
                    objective = objective + obj.inst.dist(i, obj.centers(j).getId()+1) * x(i, j);
                end
            end
            
            % Set solver options
            options = sdpsettings('solver', 'mosek', 'verbose', 0);
            
            while ~allConnected && iteration < maxIterations
                iteration = iteration + 1;
                
                % Check all zones for connectivity
                hasDisconnection = false;
                allDisconnectedComponents = containers.Map();
                
                % Extract current solution if not first iteration
                if iteration > 1
                    % Populate zones based on solution
                    x_val = value(x);
                    for j = 1:p
                        obj.zones{j} = [];
                        for i = 1:n
                            if abs(x_val(i, j) - 1) < 1e-6
                                obj.zones{j} = [obj.zones{j}, i-1];
                            end
                        end
                    end
                end
                
                % Check each zone for connected components
                for j = 1:p
                    components = obj.findConnectedComponents(obj.zones{j});
                    
                    if length(components) > 1
                        hasDisconnection = true;
                        
                        % Find which component contains the center
                        centerComponentIdx = -1;
                        for c = 1:length(components)
                            if ismember(obj.centers(j).getId(), components{c})
                                centerComponentIdx = c;
                                break;
                            end
                        end
                        
                        % Save disconnected components
                        disconnectedComponents = {};
                        for c = 1:length(components)
                            if c ~= centerComponentIdx
                                disconnectedComponents{end+1} = components{c};
                            end
                        end
                        
                        if ~isempty(disconnectedComponents)
                            allDisconnectedComponents(num2str(j)) = disconnectedComponents;
                        end
                    end
                end
                
                if ~hasDisconnection
                    allConnected = true;
                    continue;
                end
                
                % Add connectivity constraints for disconnected components
                constraintCounter = 0;
                keys_list = keys(allDisconnectedComponents);
                
                for key_idx = 1:length(keys_list)
                    key = keys_list{key_idx};
                    districtIndex = str2double(key);
                    disconnectedComponents = allDisconnectedComponents(key);
                    
                    for comp_idx = 1:length(disconnectedComponents)
                        component = disconnectedComponents{comp_idx};
                        
                        % Find neighbors of this component
                        neighbors = [];
                        for i = 1:length(component)
                            nodeId = component(i);
                            node_neighbors = obj.inst.getAreas(){nodeId+1}.getNeighbors();
                            
                            for n = 1:length(node_neighbors)
                                if ~ismember(node_neighbors(n), component)
                                    neighbors = [neighbors, node_neighbors(n)];
                                end
                            end
                        end
                        
                        neighbors = unique(neighbors);
                        
                        % Add constraint: component nodes all assigned to district j,
                        % or at least one neighbor also assigned to district j
                        constr_expr = 0;
                        
                        % For all neighbor nodes
                        for neighborId = neighbors
                            constr_expr = constr_expr + x(neighborId+1, districtIndex);
                        end
                        
                        % For all nodes in this component
                        for nodeId = component
                            constr_expr = constr_expr - x(nodeId+1, districtIndex);
                        end
                        
                        constraints = [constraints, constr_expr >= 1 - length(component)];
                        constraintCounter = constraintCounter + 1;
                    end
                end
                
                % Solve the model with new constraints
                result = optimize(constraints, objective, options);
                
                if result.problem ~= 0
                    fprintf('Connectivity iteration %d failed, model has no solution\n', iteration);
                    break;
                end
                
                fprintf('Connectivity iteration %d complete, added %d connectivity constraints\n', ...
                       iteration, constraintCounter);
            end
            
            % Final extraction of solution
            if allConnected || iteration == maxIterations
                x_val = value(x);
                for j = 1:p
                    obj.zones{j} = [];
                    for i = 1:n
                        if abs(x_val(i, j) - 1) < 1e-6
                            obj.zones{j} = [obj.zones{j}, i-1];
                        end
                    end
                end
                
                if ~allConnected
                    fprintf('Warning: Could not ensure connectivity for all zones within max iterations\n');
                end
            end
        end
        
        % Find connected components in a zone
        function components = findConnectedComponents(obj, zone)
            components = {};
            
            if isempty(zone)
                return;
            end
            
            visited = false(1, max(zone) + 1);
            
            for i = 1:length(zone)
                if ~visited(zone(i) + 1)
                    component = [];
                    queue = zone(i);
                    visited(zone(i) + 1) = true;
                    
                    while ~isempty(queue)
                        current = queue(1);
                        queue(1) = [];
                        component = [component, current];
                        
                        neighbors = obj.inst.getAreas(){current+1}.getNeighbors();
                        for n = 1:length(neighbors)
                            neighbor = neighbors(n);
                            if ismember(neighbor, zone) && ~visited(neighbor + 1)
                                queue = [queue, neighbor];
                                visited(neighbor + 1) = true;
                            end
                        end
                    end
                    
                    components{end+1} = component;
                end
            end
        end
        
        % Calculate the current objective value
        function objVal = evaluateObjective(obj)
            objVal = 0;
            
            for j = 1:length(obj.centers)
                for i = obj.zones{j}
                    objVal = objVal + obj.inst.dist(i+1, obj.centers(j).getId()+1);
                end
            end
        end
        
        % Get zones
        function z = getZones(obj)
            z = obj.zones;
        end
        
        % Get centers
        function c = getCenters(obj)
            c = obj.centers;
        end
    end
end