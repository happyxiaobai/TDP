classdef Algo < handle
    properties
        inst            % Instance object
        centers         % List of center areas
        zones           % Array of zone assignments
        r = 0.1         % Tolerance parameter
        timeLimit       % Time limit for solving
        demandUpperBound % Upper bound for demand
    end
    
    methods
        % Constructor
        function obj = Algo(instance)
            obj.inst = instance;
            obj.zones = cell(1, instance.k);
            obj.r = 0.1;
            obj.timeLimit = inf; % Default: no time limit
            
            % Calculate demand upper bound
            totalDemand = 0;
            for i = 1:instance.getN()
                totalDemand = totalDemand + instance.getAreas(){i}.getActiveness()(1);
            end
            obj.demandUpperBound = (1 + obj.r) * (totalDemand / instance.k);
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
            
            while iter < MaxIter
                % Reset centers
                obj.centers = [];
                
                % Step 1: Select initial centers using greedy random approach
                startId = randi(obj.inst.getN()) - 1; % Random start point
                obj.centers = [obj.centers, obj.inst.getAreas(){startId+1}];
                obj.inst.getAreas(){startId+1}.setCenter(true);
                
                % Continue selecting centers until we have k centers
                while length(obj.centers) < obj.inst.k
                    % Calculate minimum distance from each area to closest center
                    minDistances = inf(1, obj.inst.getN());
                    for i = 1:obj.inst.getN()
                        if ~obj.inst.getAreas(){i}.getIsCenter()
                            minDist = inf;
                            for j = 1:length(obj.centers)
                                center = obj.centers(j);
                                dist = obj.inst.dist(i, center.getId()+1);
                                if dist < minDist
                                    minDist = dist;
                                end
                            end
                            minDistances(i) = minDist;
                        end
                    end
                    
                    % Find max and min distances
                    maxMinDist = -1;
                    minMinDist = inf;
                    for i = 1:obj.inst.getN()
                        area = obj.inst.getAreas(){i};
                        if ~area.getIsCenter()
                            minDist = minDistances(i);
                            if minDist > maxMinDist
                                maxMinDist = minDist;
                            end
                            if minDist < minMinDist
                                minMinDist = minDist;
                            end
                        end
                    end
                    
                    % Calculate threshold value
                    Thre = maxMinDist - alpha * (maxMinDist - minMinDist);
                    
                    % Build candidate list
                    candidates = [];
                    for i = 1:obj.inst.getN()
                        area = obj.inst.getAreas(){i};
                        if ~area.getIsCenter() && minDistances(i) >= Thre
                            candidates = [candidates, area];
                        end
                    end
                    
                    % Select random candidate as next center
                    nextId = randi(length(candidates));
                    obj.centers = [obj.centers, candidates(nextId)];
                    obj.inst.getAreas(){candidates(nextId).getId()+1}.setCenter(true);
                end
                
                % Step 2: Solve assignment problem with the selected centers
                change = true;
                cur_value = 0.0;
                
                while change
                    change = false;
                    
                    % Solve assignment problem using centers
                    % This would typically use an optimization solver
                    % For simplicity, we're implementing a basic version
                    [zones, objVal] = obj.solveAssignmentProblem();
                    obj.zones = zones;
                    cur_value = objVal;
                    
                    % Check and adjust centers
                    for z = 1:length(obj.zones)
                        zone = obj.zones{z};
                        oldCenter = obj.centers(z).getId();
                        minDist = inf;
                        newCenter = -1;
                        
                        % Find the new center that minimizes total distance
                        for i = 1:length(zone)
                            beat = zone(i);
                            sumDist = 0.0;
                            for j = 1:length(zone)
                                if beat ~= zone(j)
                                    sumDist = sumDist + obj.inst.dist(beat+1, zone(j)+1);
                                end
                            end
                            if sumDist < minDist
                                minDist = sumDist;
                                newCenter = beat;
                            end
                        end
                        
                        % Update center if a better one is found
                        if newCenter ~= oldCenter
                            change = true;
                            obj.centers(z) = obj.inst.getAreas(){newCenter+1};
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
                for i = 1:length(obj.inst.getAreas())
                    obj.inst.getAreas(){i}.setCenter(false);
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
            zones = cell(1, length(obj.centers));
            for i = 1:length(obj.centers)
                zones{i} = [];
            end
            
            % Create assignment variables
            n = obj.inst.getN();
            p = length(obj.centers);
            x = zeros(n, p);
            
            % Set centers to be assigned to themselves
            for j = 1:p
                centerId = obj.centers(j).getId();
                x(centerId+1, j) = 1;
                zones{j} = [zones{j}, centerId];
            end
            
            % Simple greedy assignment for other areas
            for i = 1:n
                if ~ismember(i-1, [obj.centers.getId])
                    minDist = inf;
                    bestCenter = -1;
                    
                    % Find closest center that doesn't violate capacity
                    for j = 1:p
                        centerId = obj.centers(j).getId();
                        dist = obj.inst.dist(i, centerId+1);
                        
                        % Check if capacity constraint is satisfied
                        totalDemand = 0;
                        for a = 1:length(zones{j})
                            areaId = zones{j}(a);
                            totalDemand = totalDemand + obj.inst.getAreas(){areaId+1}.getActiveness()(1);
                        end
                        
                        % Add potential new area's demand
                        potentialDemand = totalDemand + obj.inst.getAreas(){i}.getActiveness()(1);
                        
                        if potentialDemand <= obj.demandUpperBound && dist < minDist
                            minDist = dist;
                            bestCenter = j;
                        end
                    end
                    
                    if bestCenter ~= -1
                        x(i, bestCenter) = 1;
                        zones{bestCenter} = [zones{bestCenter}, i-1];
                    else
                        % If no feasible assignment, assign to closest center anyway
                        [~, bestCenter] = min(obj.inst.dist(i, [obj.centers.getId]+1));
                        x(i, bestCenter) = 1;
                        zones{bestCenter} = [zones{bestCenter}, i-1];
                    end
                end
            end
            
            % Calculate objective value
            objVal = 0;
            for i = 1:n
                for j = 1:p
                    if x(i, j) == 1
                        objVal = objVal + obj.inst.dist(i, obj.centers(j).getId()+1);
                    end
                end
            end
        end
        
        % Ensure connectivity of all zones
        function [zones, allConnected] = ensureConnectivity(obj)
            zones = obj.zones;
            allConnected = true;
            
            for j = 1:length(zones)
                zone = zones{j};
                centerId = obj.centers(j).getId();
                
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
                                % For simplicity, just add nodes with neighbors in main component
                                component = components{c};
                                connected = false;
                                
                                for i = 1:length(component)
                                    nodeId = component(i);
                                    neighbors = obj.inst.getAreas(){nodeId+1}.getNeighbors();
                                    
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
        
        % Find connected components in a zone
        function components = findConnectedComponents(obj, zone)
            components = {};
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
        
        % Reassign a disconnected component to a different zone
        function reassignDisconnectedComponent(obj, component, currentZone, zones)
            % Find the closest connected zone
            bestZone = -1;
            minDist = inf;
            
            for z = 1:length(zones)
                if z ~= currentZone
                    for i = 1:length(component)
                        nodeId = component(i);
                        for j = 1:length(zones{z})
                            zoneNodeId = zones{z}(j);
                            dist = obj.inst.dist(nodeId+1, zoneNodeId+1);
                            if dist < minDist
                                minDist = dist;
                                bestZone = z;
                            end
                        end
                    end
                end
            end
            
            % Reassign the component
            if bestZone ~= -1
                zones{bestZone} = [zones{bestZone}, component];
                % Remove component from current zone
                zones{currentZone} = setdiff(zones{currentZone}, component);
            end
        end
        
        % Save results to file
        function saveResults(obj, filename, BestZones, Best, timeSpent)
            % Create output file path
            outputFilePath = ['./output/', strrep(filename, '.dat', '.txt')];
            
            % Open file for writing
            fileID = fopen(outputFilePath, 'w');
            
            % Write zone information
            for i = 1:length(BestZones)
                fprintf(fileID, 'center ID: %d\n', obj.centers(i).getId());
                for j = 1:length(BestZones{i})
                    fprintf(fileID, '%d ', BestZones{i}(j));
                end
                fprintf(fileID, '\n');
            end
            
            % Write objective value and time
            fprintf(fileID, 'best objective: %.2f\n', Best);
            fprintf(fileID, 'Runtime: %.2f s\n', timeSpent);
            
            % Close file
            fclose(fileID);
            
            fprintf('Results saved to %s\n', outputFilePath);
        end
        
        % Get solution centers for a specific scenario
        function centerIds = getCorrectSolutionCenters(obj)
            % Run algorithm with time limit
            startTime = tic;
            beta = 0.4;
            Best = inf;
            bestCenters = [];
            iter = 0;
            MaxIter = 1;
            alpha = 0;
            delta = 0.01;
            
            % Set random seed for reproducibility
            rng(42);
            
            while iter < MaxIter
                % Reset centers
                obj.centers = [];
                
                % Select initial centers using greedy random approach
                startId = randi(obj.inst.getN()) - 1;
                obj.centers = [obj.centers, obj.inst.getAreas(){startId+1}];
                obj.inst.getAreas(){startId+1}.setCenter(true);
                
                % Continue selecting centers until we have k centers
                while iter < MaxIter
                
                % Step 2: Solve assignment problem with the selected centers
                change = true;
                cur_value = 0.0;
                
                while change
                    % Solve assignment problem and adjust centers
                    % ...
                end
                
                % Check if solution is better than current best
                if cur_value < Best
                    Best = cur_value;
                    bestCenters = [obj.centers.getId];
                end
                
                iter = iter + 1;
            end
            
            % If not enough centers, add random ones
            if length(bestCenters) < obj.inst.k
                availableAreas = setdiff(0:obj.inst.getN()-1, bestCenters);
                while length(bestCenters) < obj.inst.k && ~isempty(availableAreas)
                    randIdx = randi(length(availableAreas));
                    bestCenters = [bestCenters, availableAreas(randIdx)];
                    availableAreas(randIdx) = [];
                end
            end
            
            centerIds = bestCenters;
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