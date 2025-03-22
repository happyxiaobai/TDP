% DistributionallyRobustMain.m
function DistributionallyRobustMain()
    % Set parameters
    instanceFile = './Instances/2DU60-05-1.dat';  % Input file path
    outputFileName = '2DU60-05-1_drcc';           % Output file name
    
    % Set distributionally robust optimization parameters
    gamma = 0.05;      % Risk parameter
    numScenarios = 100; % Number of scenarios
    seed = 12345678;   % Random seed
    
    % Demand parameters
    E = 50.0;          % Expected value
    RSDValues = [0.125, 0.25, 0.5]; % Relative standard deviation array
    
    % Select specific RSD value
    RSD = RSDValues(1);
    
    % Other DRCC parameters
    useD1 = true;      % Whether to use D1 fuzzy set
    delta1 = 2;        % D2 fuzzy set parameter
    delta2 = 4;        % D2 fuzzy set parameter
    useJointChance = false; % Whether to use joint chance constraint
    
    % Load instance
    instance = Instance(instanceFile);
    
    % Generate random scenarios using uniform distribution
    scenarios = generateScenarios(instance.getN(), numScenarios, E, RSD, seed);
    
    % Create and run the distributionally robust algorithm
    algo = DistributionallyRobustAlgo(instance, scenarios, gamma, seed, useD1, delta1, delta2, useJointChance);
    
    % Run algorithm
    algo.run(outputFileName);
    
    fprintf('Distributionally robust chance-constrained district design problem solved.\n');
    
    % Visualize results
    outputImagePath = ['./output/', outputFileName, '_visualization.png'];
    visualizer = DistrictVisualizer(instance, algo.getZones(), algo.getCenters());
    visualizer.saveVisualization(outputImagePath);
    
    fprintf('Visualization saved to: %s\n', outputImagePath);
end

% Function to generate scenarios
function scenarios = generateScenarios(n, numScenarios, E, RSD, seed)
    scenarios = zeros(numScenarios, n);
    
    % Set random seed
    rng(seed);
    
    % Calculate uniform distribution endpoints
    lowerBound = E * (1 - sqrt(3) * RSD);
    upperBound = E * (1 + sqrt(3) * RSD);
    
    % Generate scenarios
    for s = 1:numScenarios
        for i = 1:n
            % Generate demand from uniform distribution
            demand = lowerBound + rand() * (upperBound - lowerBound);
            
            % Ensure demand is positive
            scenarios(s, i) = max(1, demand);
        end
    end
end