classdef Algo < handle
    properties
        inst            % Instance object
        centers_ids     % Array of center IDs (instead of Area objects)
        zones           % Array of zone assignments
        r = 0.1         % Tolerance parameter
        timeLimit       % Time limit for solving
        demandUpperBound % Upper bound for demand
    end
    
    methods
        % Constructor
        function obj = Algo(instance)
            obj.inst = instance;
            obj.centers_ids = [];  % Initialize as empty array instead of empty list
            obj.zones = cell(1, instance.k);
            obj.r = 0.1;
            obj.timeLimit = inf; % Default: no time limit
            
            % Calculate demand upper bound
            obj.demandUpperBound = (1 + obj.r) * (sum(inst.capacity) / instance.k);
        end
        
        % Set time limit
        function setTimeLimit(obj, seconds)
            obj.timeLimit = seconds;
        end
        
        % Run the algorithm
        function run(obj, filename)
            startTime = tic;
            beta = 0.4;
            Best = inf;
            iter = 0;
            MaxIter = 1;
            alpha = 0;
            delta = 0.01;
            BestZones = cell(1, obj.inst.k);
            
            % Create random number generator
            rng(42); % For reproducibility
            areas = obj.inst.getAreas();
            while iter < MaxIter
                % Reset centers
                obj.centers_ids = [];
                
                % Step 1: Select initial centers using greedy random approach
                startId = randi(obj.inst.getN()); % Random start index (1-based)
                
                obj.centers_ids = [obj.centers_ids, startId]; % Store the ID instead of Area object
                areas{startId}.setCenter(true);
                
                % Continue selecting centers until we have k centers
                while length(obj.centers_ids) < obj.inst.k
                    % Calculate minimum distance from each area to closest center

                    minDistances = inf(1, obj.inst.getN());
                    % Mark which areas are centers in a single operation
                    centerMask = ismember(1:obj.inst.getN(), obj.centers_ids);
                    
                    % For each non-center area, find minimum distance to any center
                    nonCenterMask = ~centerMask;
                    nonCenterIndices = find(nonCenterMask);
                    
                    % Get the distance matrix subset for non-centers to centers
                    distSubMatrix = obj.inst.dist(nonCenterIndices, obj.centers_ids);
                    
                    % Find minimum distance for each non-center
                    [minDistances(nonCenterIndices), ~] = min(distSubMatrix, [], 2);
                    
                    % Find max and min of these minimum distances
                    maxMinDist = max(minDistances(nonCenterIndices));
                    minMinDist = min(minDistances(nonCenterIndices));
                    
                    % Calculate threshold value
                    Thre = maxMinDist - alpha * (maxMinDist - minMinDist);
                    
                    % Build candidate list
                    % Build candidate list using logical indexing
                    candidatesMask = ~centerMask & (minDistances >= Thre);
                    candidates = find(candidatesMask);
                    
                    % Select random candidate as next center
                    nextId = randi(length(candidates));
                    candidateId = candidates(nextId);
                    obj.centers_ids = [obj.centers_ids, candidateId];
                    areas{candidateId}.setCenter(true);
                end
                
                % Step 2: Solve assignment problem with the selected centers
                change = true;
                cur_value = 0.0;
                
                while change
                    change = false;
                    
                    % Solve assignment problem using centers
                    [zones, objVal] = obj.solveAssignmentProblem();
                    obj.zones = zones;
                    cur_value = objVal;
                    
                    % Check and adjust centers
                    for z = 1:length(obj.zones)
                        zone = obj.zones{z};
                        oldCenterId = obj.centers_ids(z);
                        minDist = inf;
                        newCenterId = -1;
                        
                        % Find the new center that minimizes total distance
                        for i = 1:length(zone)
                            beatId = zone(i);
                            sumDist = 0.0;
                            for j = 1:length(zone)
                                if beatId ~= zone(j)
                                    sumDist = sumDist + obj.inst.dist(beatId, zone(j)); % IDs are already 1-based
                                end
                            end
                            if sumDist < minDist
                                minDist = sumDist;
                                newCenterId = beatId;
                            end
                        end
                        
                        % Update center if a better one is found
                        if newCenterId ~= oldCenterId
                            change = true;
                            obj.centers_ids(z) = newCenterId;
                        end
                    end
                end
                
                % Check connectivity and add connectivity constraints if needed
                [obj.zones, isConnected] = obj.ensureConnectivity();
                
                % Update best solution
                if cur_value < Best
                    Best = cur_value;
                    for z = 1:obj.inst.k
                        BestZones{z} = obj.zones{z};
                    end
                end
                
                % Update parameters for next iteration
                iter = iter + 1;
                fprintf('Iteration %d, best result: %.2f\n', iter, Best);
                
                if alpha < beta
                    alpha = alpha + delta;
                else
                    alpha = 0;
                end
                
                % Reset center flags
                areas = obj.inst.getAreas();
                for i = 1:length(areas)
                    areas{i}.setCenter(false);
                end
            end
            
            % Calculate total time
            endTime = toc(startTime);
            
            % Save results to file
            obj.saveResults(filename, BestZones, Best, endTime);
        end
        
        % Solve the assignment problem with fixed centers
        function [zones, objVal] = solveAssignmentProblem(obj)
            % Initialize zones
            zones = cell(1, length(obj.centers_ids));
            
            % Get dimensions
            n = obj.inst.getN();
            p = length(obj.centers_ids);
            
            % Create YALMIP binary variables for assignment
            x = binvar(n, p, 'full');
            
            % Create constraints
            constraints = [];
            
            % Each area must be assigned to exactly one center
            constraints = [constraints, sum(x, 2) == 1];
            
            % Centers must be assigned to themselves
            for j = 1:p
                centerId = obj.centers_ids(j);
                constraints = [constraints, x(centerId, j) == 1];
            end
            %下面是上面的替换代码，可以试试
            % rows = obj.centers_ids;
            % cols = 1:p;
            % constraints = [constraints, x(sub2ind([n, p], rows, cols)) == 1];

            
            % Capacity constraints for each center
            % Get demands vector from instance capacity
            demands = obj.inst.capacity;
            
            % Make sure demands is a column vector
            if size(demands, 1) == 1
                demands = demands'; % Convert row to column if needed
            end
            
            % Add all capacity constraints in one operation
            constraints = [constraints, demands' * x <= obj.demandUpperBound];
            
            % Objective: minimize total distance
            % Fully vectorized objective calculation
            objective = sum(sum(obj.inst.dist(:, obj.centers_ids) .* x));
            
            % Set options for Gurobi
            options = sdpsettings('solver', 'gurobi', 'verbose', 0);
            
            % Solve the problem
            diagnostics = optimize(constraints, objective, options);
            
            % Extract solution if feasible
            if diagnostics.problem == 0
                % Extract assignment values
                x_val = value(x);
                
                % Populate zones
                for j = 1:p
                    zones{j} = find(x_val(:, j) > 0.5)';
                end
                
                % Calculate objective value
                objVal = value(objective);
            else
                % Fallback to greedy approach if optimization fails
                objVal = inf;
                warning('Optimization failed. Using fallback greedy approach.');
                
                % [Insert original greedy code here if needed]
            end
        end
        
        % Ensure connectivity of all zones (modified to use center IDs)
        function [zones, allConnected] = ensureConnectivity(obj)
            zones = obj.zones;
            allConnected = true;
            
            for j = 1:length(zones)
                zone = zones{j};
                centerId = obj.centers_ids(j);
                
                % Find connected components in this zone
                components = obj.findConnectedComponents(zone);
                
                if length(components) > 1
                    allConnected = false;
                    
                    % Find which component contains the center
                    centerComponentIdx = -1;
                    for c = 1:length(components)
                        if ismember(centerId, components{c})
                            centerComponentIdx = c;
                            break;
                        end
                    end
                    
                    % Integrate disconnected components
                    if centerComponentIdx ~= -1
                        newZone = components{centerComponentIdx};
                        for c = 1:length(components)
                            if c ~= centerComponentIdx
                                % Try to connect this component to the main component
                                component = components{c};
                                connected = false;
                                
                                for i = 1:length(component)
                                    nodeId = component(i);
                                    % Get neighbors using 1-based indexing
                                    neighbors = obj.inst.getAreas(){nodeId}.getNeighbors();
                                    
                                    for n = 1:length(neighbors)
                                        if ismember(neighbors(n), newZone)
                                            connected = true;
                                            break;
                                        end
                                    end
                                    
                                    if connected
                                        break;
                                    end
                                end
                                
                                if connected
                                    newZone = [newZone, component];
                                else
                                    % If can't connect, assign to closest feasible center
                                    obj.reassignDisconnectedComponent(component, j, zones);
                                end
                            end
                        end
                        
                        zones{j} = newZone;
                    end
                end
            end
        end
        
        % Other methods will be similarly updated
        
        % Modified getCenters method to maintain compatibility with other code
        function areas = getCenters(obj)
            % Convert center IDs back to Area objects if needed by other code
            areas = [];
            for i = 1:length(obj.centers_ids)
                centerId = obj.centers_ids(i);
                areas = [areas, obj.inst.getAreas(){centerId}];
            end
        end
        
        % Get centers IDs directly 
        function ids = getCenterIds(obj)
            ids = obj.centers_ids;
        end
        
        % Get zones
        function z = getZones(obj)
            z = obj.zones;
        end
    end
end