classdef Instance < handle
    properties
        n               % Number of areas
        areas           % Array of Area objects
        edges           % Adjacency matrix
        average1        % Average of first activeness metric
        average2        % Average of second activeness metric
        k               % Number of districts to create
        dist            % Distance matrix
    end
    
    methods
        % Constructor
        function obj = Instance(filepath)
            if nargin > 0
                obj.loadFromFile(filepath);
            end
        end
        
        % Load instance data from file
        function loadFromFile(obj, filepath)
            % Open the file
            fileID = fopen(filepath, 'r');
            if fileID == -1
                error('Cannot open file: %s', filepath);
            end
            
            % Read number of areas
            obj.n = fscanf(fileID, '%d', 1);
            
            % Initialize areas array
            obj.areas = cell(1, obj.n);
            
            % Read area data
            sum1 = 0;
            sum2 = 0;
            for i = 1:obj.n
                id = fscanf(fileID, '%d', 1);
                x = fscanf(fileID, '%f', 1);
                y = fscanf(fileID, '%f', 1);
                
                activeness = zeros(1, 3);
                for j = 1:3
                    activeness(j) = fscanf(fileID, '%f', 1);
                end
                
                sum1 = sum1 + activeness(1);
                sum2 = sum2 + activeness(2);
                
                obj.areas{i} = Area(id, x, y, activeness);
            end
            
            % Initialize edges matrix
            obj.edges = zeros(obj.n, obj.n);
            
            % Read number of edges
            m = fscanf(fileID, '%d', 1);
            
            % Read edge data
            for i = 1:m
                a = fscanf(fileID, '%d', 1) + 1; % +1 for 1-indexing in MATLAB
                b = fscanf(fileID, '%d', 1) + 1; % +1 for 1-indexing in MATLAB
                
                obj.areas{a}.addNeighbor(b-1); % -1 to keep original IDs
                obj.areas{b}.addNeighbor(a-1); % -1 to keep original IDs
                
                obj.edges(a, b) = 1;
                obj.edges(b, a) = 1;
            end
            
            % Read k (number of districts)
            obj.k = fscanf(fileID, '%d', 1);
            
            % Calculate averages
            obj.average1 = sum1 / obj.k;
            obj.average2 = sum2 / obj.k;
            
            % Close the file
            fclose(fileID);
            
            % Generate distance matrix
            obj.generateDistanceMatrix();
        end
        
        % Create a clone of the current instance
        function cloned = clone(obj)
            cloned = Instance();
            cloned.n = obj.n;
            cloned.k = obj.k;
            cloned.average1 = obj.average1;
            cloned.average2 = obj.average2;
            
            % Copy areas
            cloned.areas = cell(1, obj.n);
            for i = 1:obj.n
                area = obj.areas{i};
                cloned.areas{i} = Area(area.getId(), area.getX(), area.getY(), area.getActiveness());
                cloned.areas{i}.setCenter(area.getIsCenter());
                
                % Copy neighbors
                for neighbor = area.getNeighbors()
                    cloned.areas{i}.addNeighbor(neighbor);
                end
            end
            
            % Copy edges
            cloned.edges = obj.edges;
            
            % Copy distance matrix
            cloned.dist = obj.dist;
        end
        
        % Create instance with specific scenario demands
        function inst = createScenarioInstance(obj, scenarioDemands)
            inst = obj.clone();
            sum1 = 0;
            sum2 = 0;
            
            for i = 1:obj.n
                area = obj.areas{i};
                origActiveness = area.getActiveness();
                newActiveness = origActiveness;
                newActiveness(1) = scenarioDemands(i);
                
                inst.areas{i}.setActiveness(newActiveness);
                sum1 = sum1 + newActiveness(1);
                sum2 = sum2 + newActiveness(2);
            end
            
            inst.average1 = sum1 / inst.k;
            inst.average2 = sum2 / inst.k;
        end
        
        % Generate distance matrix
        function generateDistanceMatrix(obj)
            obj.dist = zeros(obj.n, obj.n);
            for i = 1:obj.n
                for j = i+1:obj.n
                    area_i = obj.areas{i};
                    area_j = obj.areas{j};
                    distance = sqrt((area_i.getX() - area_j.getX())^2 + ...
                                   (area_i.getY() - area_j.getY())^2);
                    obj.dist(i, j) = distance;
                    obj.dist(j, i) = distance;
                end
            end
        end
        
        % Getters
        function value = getN(obj)
            value = obj.n;
        end
        
        function value = getAreas(obj)
            value = obj.areas;
        end
        
        function value = getEdges(obj)
            value = obj.edges;
        end
        
        % Output instance information
        function output(obj)
            fprintf('Number of areas: %d\n', obj.n);
            fprintf('Area information:\n');
            for i = 1:length(obj.areas)
                area = obj.areas{i};
                activeness = area.getActiveness();
                fprintf('Area ID: %d, x: %.2f, y: %.2f, Activeness: [%.2f, %.2f, %.2f]\n', ...
                    area.getId(), area.getX(), area.getY(), ...
                    activeness(1), activeness(2), activeness(3));
            end
            
            fprintf('Edge information:\n');
            for i = 1:obj.n
                for j = 1:obj.n
                    fprintf('%d ', obj.edges(i, j));
                end
                fprintf('\n');
            end
        end
    end
end