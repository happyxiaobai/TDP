
% DistributionallyRobustMain.m
function DistributionallyRobustMain()
    clc;clear;
    % Start timer for overall execution
    totalStartTime = tic;
    timerActive = false;
    timeoutTimer = [];
    
    try
        % Set parameters
        instanceFile = './Instances/2DU60-05-1.dat';  % Input file path
        outputFileName = '2DU60-05-1_drcc';           % Output file name
        outputDir = './output/';
        
        % Set distributionally robust optimization parameters
        gamma = 0.05;      % Risk parameter
        numScenarios = 10; % Number of scenarios
        seed = 12345678;   % Random seed
        
        % Demand parameters
        E = 50.0;          % Expected value
        RSDValues = [0.125, 0.25, 0.5]; % Relative standard deviation array
        
        % Select specific RSD value
        RSD = RSDValues(1);
        
        % Other DRCC parameters
        useD1 = true;      % Whether to use D1 fuzzy set
        delta1 = 1;        % D2 fuzzy set parameter
        delta2 = 2;        % D2 fuzzy set parameter
        useJointChance = false; % Whether to use joint chance constraint
        
        % Algorithm timeout (in seconds)
        timeout = 3600; % 1 hour timeout
        
        % Display configuration
        displayConfig(instanceFile, outputFileName, gamma, numScenarios, E, RSD, useD1, delta1, delta2, useJointChance);
        
        % Validate parameters
        validateParams(gamma, numScenarios, E, RSD, delta1, delta2);
        
        % Ensure output directory exists
        if ~exist(outputDir, 'dir')
            [success, msg] = mkdir(outputDir);
            if ~success
                error('Failed to create directory %s: %s', outputDir, msg);
            end
            fprintf('Created output directory: %s\n', outputDir);
        end
        
        % Check if instance file exists
        if ~exist(instanceFile, 'file')
            error('Instance file not found: %s', instanceFile);
        end
        
        % Load instance
        fprintf('Loading instance from %s...\n', instanceFile);
        instance = Instance(instanceFile);
        
        if isempty(instance) || instance.getN() == 0
            error('Failed to load instance or instance has no areas');
        end
        
        fprintf('Instance contains %d areas\n', instance.getN());
        
        % Generate random scenarios using vectorized operations
        fprintf('Generating %d scenarios...\n', numScenarios);
        scenarios = generateScenarios(instance.getN(), numScenarios, E, RSD, seed);
        
        % Create and run the distributionally robust algorithm
        fprintf('Initializing algorithm...\n');
        algo = DistributionallyRobustAlgo(instance, scenarios, gamma, seed, useD1, delta1, delta2, useJointChance);
        
        if isempty(algo)
            error('Failed to create algorithm object');
        end
        
        % Try to set algorithm timeout
        try
            if isfield(algo, 'timeLimit') || isprop(algo, 'timeLimit')
                algo.timeLimit = timeout;
                fprintf('Algorithm timeout set to %d seconds\n', timeout);
            end
        catch
            fprintf('Warning: Could not set algorithm timeout\n');
        end
        
        % Run algorithm with timeout monitoring
        fprintf('\nRunning algorithm...\n');
        algoRunStartTime = tic;
        
        % Try to create a timer for timeout monitoring
        try
            if exist('timer', 'file') && exist('timerfind', 'file')
                % Delete any existing timers with the same name
                existingTimers = timerfind('Name', 'TimeoutTimer');
                if ~isempty(existingTimers)
                    delete(existingTimers);
                end
                
                % Create a new timer
                timeoutTimer = timer('TimerFcn', @(~,~) timeoutCheck(algoRunStartTime, timeout), ...
                       'ExecutionMode', 'fixedRate', ...
                       'Period', 10, ... % Check every 10 seconds
                       'Name', 'TimeoutTimer');
                start(timeoutTimer);
                timerActive = true;
            end
        catch timerError
            fprintf('Warning: Could not create timeout timer: %s\n', timerError.message);
        end
        
        % Run algorithm
        algo.run(outputFileName);
        
        % Stop the timer if it's active
        cleanupTimer();
        
        fprintf('Distributionally robust chance-constrained district design problem solved.\n');
        
        % Get results from the algorithm
        zones = algo.getZones();
        centers = algo.getCenters();
        
        % Visualize results
        outputImagePath = fullfile(outputDir, [outputFileName, '_visualization.png']);
        fprintf('Creating visualization...\n');
        visualizer = DistrictVisualizer(instance, zones, centers);
        visualizer.saveVisualization(outputImagePath);
        
        fprintf('Visualization saved to: %s\n', outputImagePath);
        
        % Display solution statistics
        displayStats(instance, zones, centers);
        
        % Total runtime
        totalRunTime = toc(totalStartTime);
        fprintf('\nTotal run time: %.2f seconds (%.2f minutes)\n', ...
            totalRunTime, totalRunTime/60);
        
    catch e
        % Stop the timer if it's active
        cleanupTimer();
        
        fprintf('\n\n');
        fprintf('Error occurred: %s\n', e.message);
        fprintf('Stack trace:\n');
        for i = 1:length(e.stack)
            fprintf('  File: %s, Line: %d, Function: %s\n', ...
                e.stack(i).file, e.stack(i).line, e.stack(i).name);
        end
    end
    
    % Nested function to clean up timer
    function cleanupTimer()
        if timerActive && ~isempty(timeoutTimer) && isvalid(timeoutTimer)
            try
                stop(timeoutTimer);
                delete(timeoutTimer);
            catch
                % Ignore errors in timer cleanup
            end
            timerActive = false;
        end
    end
end

% Function to display configuration
function displayConfig(instanceFile, outputFileName, gamma, numScenarios, E, RSD, useD1, delta1, delta2, useJointChance)
    fprintf('----------------------------------------\n');
    fprintf('Distributionally Robust Optimization Configuration:\n');
    fprintf('----------------------------------------\n');
    fprintf('Instance file: %s\n', instanceFile);
    fprintf('Output file: %s\n', outputFileName);
    fprintf('Risk parameter (gamma): %.4f\n', gamma);
    fprintf('Number of scenarios: %d\n', numScenarios);
    fprintf('Expected demand value (E): %.2f\n', E);
    fprintf('Relative standard deviation (RSD): %.4f\n', RSD);
    
    if useD1
        fprintf('Fuzzy set type: D1\n');
    else
        fprintf('Fuzzy set type: D2\n');
        fprintf('Delta1 parameter: %.2f\n', delta1);
        fprintf('Delta2 parameter: %.2f\n', delta2);
    end
    
    if useJointChance
        fprintf('Constraint type: Joint chance constraint\n');
    else
        fprintf('Constraint type: Individual chance constraint\n');
    end
    fprintf('----------------------------------------\n\n');
end

% Function to validate parameters
function validateParams(gamma, numScenarios, E, RSD, delta1, delta2)
    % Check gamma (risk parameter)
    if ~isnumeric(gamma) || ~isscalar(gamma) || gamma <= 0 || gamma >= 1
        error('Parameter "gamma" must be a scalar between 0 and 1');
    end
    
    % Check numScenarios
    if ~isnumeric(numScenarios) || ~isscalar(numScenarios) || numScenarios <= 0 || mod(numScenarios, 1) ~= 0
        error('Parameter "numScenarios" must be a positive integer');
    end
    
    % Check E (expected value)
    if ~isnumeric(E) || ~isscalar(E) || E <= 0
        error('Parameter "E" must be a positive scalar');
    end
    
    % Check RSD (relative standard deviation)
    if ~isnumeric(RSD) || ~isscalar(RSD) || RSD <= 0
        error('Parameter "RSD" must be a positive scalar');
    end
    
    % Check delta1 and delta2 (D2 fuzzy set parameters)
    if ~isnumeric(delta1) || ~isscalar(delta1) || delta1 <= 0
        error('Parameter "delta1" must be a positive scalar');
    end
    
    if ~isnumeric(delta2) || ~isscalar(delta2) || delta2 <= 0
        error('Parameter "delta2" must be a positive scalar');
    end
    
    if delta2 <= delta1
        error('Parameter "delta2" must be greater than "delta1"');
    end
    
    % Check if lower bound would be negative
    lowerBound = E * (1 - sqrt(3) * RSD);
    if lowerBound <= 0
        warning(['With E=%.2f and RSD=%.4f, the calculated lower bound (%.2f) is non-positive. ' ...
                'This will be adjusted to 1 for demand generation.'], E, RSD, lowerBound);
    end
end

% Function to generate scenarios - vectorized version
function scenarios = generateScenarios(n, numScenarios, E, RSD, seed)
    % Set random seed for reproducibility
    rng(seed);
    
    % Calculate uniform distribution endpoints
    lowerBound = E * (1 - sqrt(3) * RSD);
    upperBound = E * (1 + sqrt(3) * RSD);
    
    % Handle potential negative lower bound
    if lowerBound <= 0
        warning('Lower bound for demand is non-positive. Setting to 1.');
        lowerBound = 1;
    end
    
    % Initialize scenarios matrix
    scenarios = zeros(numScenarios, n);
    
    % For very large instances, generate in chunks to save memory
    maxChunkSize = 1000; % Adjust based on available memory
    
    % If matrix is very large, generate in chunks
    if numScenarios * n > 10^7 % Threshold for chunk processing
        fprintf('Large instance detected. Generating scenarios in chunks...\n');
        
        for chunkStart = 1:maxChunkSize:numScenarios
            % Determine chunk end
            chunkEnd = min(chunkStart + maxChunkSize - 1, numScenarios);
            chunkSize = chunkEnd - chunkStart + 1;
            
            % Generate random values for this chunk
            randomValues = rand(chunkSize, n);
            
            % Scale to the desired range
            scenarios(chunkStart:chunkEnd, :) = lowerBound + randomValues * (upperBound - lowerBound);
            
            % Report progress
            fprintf('Generated scenarios %d to %d of %d\n', chunkStart, chunkEnd, numScenarios);
        end
    else
        % For smaller instances, generate all at once
        randomValues = rand(numScenarios, n);
        scenarios = lowerBound + randomValues * (upperBound - lowerBound);
    end
    
    % Ensure all demands are positive (at least 1)
    scenarios = max(1, scenarios);
end

% Function to check if algorithm execution has exceeded timeout
function timeoutCheck(startTime, timeout)
    elapsedTime = toc(startTime);
    if elapsedTime > timeout
        % This will be caught by the try-catch block in the main function
        error('Algorithm execution exceeded timeout of %d seconds', timeout);
    end
end

% Function to display summary statistics
function displayStats(instance, zones, centers)
    fprintf('\n----------------------------------------\n');
    fprintf('Solution Summary Statistics:\n');
    fprintf('----------------------------------------\n');
    fprintf('Number of areas: %d\n', instance.getN());
    fprintf('Number of districts: %d\n', length(zones));
    
    % Calculate district sizes
    districtSizes = zeros(1, length(zones));
    for i = 1:length(zones)
        districtSizes(i) = length(zones{i});
    end
    
    fprintf('\nDistrict sizes (number of areas):\n');
    fprintf('  Min: %d\n', min(districtSizes));
    fprintf('  Max: %d\n', max(districtSizes));
    fprintf('  Mean: %.2f\n', mean(districtSizes));
    fprintf('  Standard deviation: %.2f\n', std(districtSizes));
    
    % Calculate total demand per district if capacity information is available
    if ~isempty(instance.getCapacity())
        capacity = instance.getCapacity();
        districtDemands = zeros(1, length(zones));
        
        for i = 1:length(zones)
            % Extract zone areas and sum their demands
            zoneAreas = zones{i};
            districtDemands(i) = sum(capacity(zoneAreas));
        end
        
        fprintf('\nDistrict demands:\n');
        fprintf('  Min: %.2f\n', min(districtDemands));
        fprintf('  Max: %.2f\n', max(districtDemands));
        fprintf('  Mean: %.2f\n', mean(districtDemands));
        fprintf('  Standard deviation: %.2f\n', std(districtDemands));
    end
    
    fprintf('----------------------------------------\n');
end