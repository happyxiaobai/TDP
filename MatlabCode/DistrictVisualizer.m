classdef DistrictVisualizer < handle
    properties
        instance
        zones
        centers
        width = 1200
        height = 800
        mainX = 10
        mainY = 50
        mainWidth = 980
        mainHeight = 700
        padding = 80
        minX
        maxX
        minY
        maxY
        scaleX
        scaleY
        areaPositions
    end
    
    methods
        % Constructor
        function obj = DistrictVisualizer(instance, zones, centers)
            obj.instance = instance;
            obj.zones = zones;
            obj.centers = centers;
            obj.areaPositions = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            
            % Calculate bounds for scaling
            areas = instance.getAreas();
            obj.minX = inf;
            obj.maxX = -inf;
            obj.minY = inf;
            obj.maxY = -inf;
            
            for i = 1:length(areas)
                area = areas{i};
                obj.minX = min(obj.minX, area.getX());
                obj.maxX = max(obj.maxX, area.getX());
                obj.minY = min(obj.minY, area.getY());
                obj.maxY = max(obj.maxY, area.getY());
            end
            
            % Calculate scaling factors
            obj.scaleX = (obj.mainWidth - 2 * obj.padding) / (obj.maxX - obj.minX);
            obj.scaleY = (obj.mainHeight - 2 * obj.padding) / (obj.maxY - obj.minY);
            
            % Generate initial positions
            for i = 1:length(areas)
                area = areas{i};
                x = obj.mainX + obj.padding + (area.getX() - obj.minX) * obj.scaleX;
                y = obj.mainY + obj.padding + (area.getY() - obj.minY) * obj.scaleY;
                obj.areaPositions(area.getId()) = [x, y];
            end
            
            % Adjust positions to avoid overlaps
            obj.adjustPositionsToAvoidOverlaps();
            
            % Ensure all nodes are inside borders
            obj.ensureNodesInsideBorders();
        end
        
        % Ensure all nodes are inside borders
        function ensureNodesInsideBorders(obj)
            buffer = 25; % Minimum distance from edge
            
            keys_list = keys(obj.areaPositions);
            for i = 1:length(keys_list)
                key = str2double(keys_list{i});
                pos = obj.areaPositions(key);
                
                x = max(obj.mainX + buffer, min(obj.mainX + obj.mainWidth - buffer, pos(1)));
                y = max(obj.mainY + buffer, min(obj.mainY + obj.mainHeight - buffer, pos(2)));
                
                obj.areaPositions(key) = [x, y];
            end
        end
        
        % Adjust positions to avoid overlaps
        function adjustPositionsToAvoidOverlaps(obj)
            areas = obj.instance.getAreas();
            nodeRadius = 8;
            minDistance = nodeRadius * 4; % Minimum distance between nodes
            
            % Use force-directed positioning to separate overlapping nodes
            hasOverlap = true;
            iterations = 0;
            maxIterations = 150;
            
            while hasOverlap && iterations < maxIterations
                hasOverlap = false;
                iterations = iterations + 1;
                
                keys_list = keys(obj.areaPositions);
                
                for i = 1:length(keys_list)
                    id1 = str2double(keys_list{i});
                    p1 = obj.areaPositions(id1);
                    
                    for j = i+1:length(keys_list)
                        id2 = str2double(keys_list{j});
                        p2 = obj.areaPositions(id2);
                        
                        % Calculate distance between nodes
                        dx = p2(1) - p1(1);
                        dy = p2(2) - p1(2);
                        distance = sqrt(dx^2 + dy^2);
                        
                        % If nodes are too close, push them apart
                        if distance < minDistance
                            hasOverlap = true;
                            
                            % Calculate repulsion force
                            force = minDistance - distance;
                            fx = force * dx / distance;
                            fy = force * dy / distance;
                            
                            % Apply force to both nodes in opposite directions
                            p2(1) = p2(1) + fx/2;
                            p2(2) = p2(2) + fy/2;
                            p1(1) = p1(1) - fx/2;
                            p1(2) = p1(2) - fy/2;
                            
                            % Keep nodes within main area
                            p1(1) = max(obj.mainX + obj.padding, min(obj.mainX + obj.mainWidth - obj.padding, p1(1)));
                            p1(2) = max(obj.mainY + obj.padding, min(obj.mainY + obj.mainHeight - obj.padding, p1(2)));
                            p2(1) = max(obj.mainX + obj.padding, min(obj.mainX + obj.mainWidth - obj.padding, p2(1)));
                            p2(2) = max(obj.mainY + obj.padding, min(obj.mainY + obj.mainHeight - obj.padding, p2(2)));
                            
                            obj.areaPositions(id1) = p1;
                            obj.areaPositions(id2) = p2;
                        end
                    end
                end
            end
            
            % Apply attraction forces between connected nodes
            for iter = 1:60
                areas = obj.instance.getAreas();
                
                for i = 1:length(areas)
                    area = areas{i};
                    id1 = area.getId();
                    p1 = obj.areaPositions(id1);
                    
                    for j = 1:length(area.getNeighbors())
                        neighborId = area.getNeighbors()(j);
                        p2 = obj.areaPositions(neighborId);
                        
                        % Calculate distance
                        dx = p2(1) - p1(1);
                        dy = p2(2) - p1(2);
                        distance = sqrt(dx^2 + dy^2);
                        
                        % If connected nodes are too far, pull them closer
                        if distance > minDistance * 4
                            force = (distance - minDistance * 3) / 10;
                            fx = force * dx / distance;
                            fy = force * dy / distance;
                            
                            % Apply gentle attraction
                            p1(1) = p1(1) + fx;
                            p1(2) = p1(2) + fy;
                            p2(1) = p2(1) - fx;
                            p2(2) = p2(2) - fy;
                            
                            % Keep within bounds
                            p1(1) = max(obj.mainX + obj.padding, min(obj.mainX + obj.mainWidth - obj.padding, p1(1)));
                            p1(2) = max(obj.mainY + obj.padding, min(obj.mainY + obj.mainHeight - obj.padding, p1(2)));
                            p2(1) = max(obj.mainX + obj.padding, min(obj.mainX + obj.mainWidth - obj.padding, p2(1)));
                            p2(2) = max(obj.mainY + obj.padding, min(obj.mainY + obj.mainHeight - obj.padding, p2(2)));
                            
                            obj.areaPositions(id1) = p1;
                            obj.areaPositions(neighborId) = p2;
                        end
                    end
                end
            end
        end
        
        % Save visualization to file
        function saveVisualization(obj, outputPath)
            % Create figure
            figure('Visible', 'off', 'Position', [100, 100, obj.width, obj.height]);
            
            % Set background color
            set(gcf, 'Color', [248, 248, 252]/255);
            
            % Draw main area with gradient
            rectangle('Position', [obj.mainX, obj.mainY, obj.mainWidth, obj.mainHeight], ...
                     'FaceColor', [240, 242, 245]/255, 'EdgeColor', [80, 80, 100]/255, 'LineWidth', 2);
            
            % Generate colors for each district
            districtColors = obj.generateDistrictColors(length(obj.zones));
            
            % Draw title
            title(['District Visualization - ', num2str(length(obj.zones)), ' districts'], ...
                 'FontSize', 16, 'FontWeight', 'bold');
            
            % Draw legend
            obj.drawLegend(districtColors);
            
            % Set axis limits
            axis([0 obj.width 0 obj.height]);
            
            % Clip to main area
            rectangle('Position', [obj.mainX, obj.mainY, obj.mainWidth, obj.mainHeight], ...
                     'Clipping', 'on');
            
            % Draw connections between adjacent areas
            obj.drawConnections();
            
            % Draw areas colored by district
            obj.drawAreas(districtColors);
            
            % Highlight centers
            obj.drawCenters();
            
            % Save to file
            saveas(gcf, outputPath);
            fprintf('Visualization saved to: %s\n', outputPath);
            
            % Close figure
            close(gcf);
        end
        
        % Generate colors for districts
        function colors = generateDistrictColors(obj, numDistricts)
            colors = cell(1, numDistricts);
            
            % Enhanced color palette
            palette = {
                [45, 125, 210]/255,   % blue
                [255, 112, 10]/255,   % orange
                [35, 170, 85]/255,    % green
                [220, 55, 60]/255,    % red
                [150, 100, 205]/255,  % purple
                [128, 80, 60]/255,    % brown
                [230, 90, 183]/255,   % pink
                [80, 80, 90]/255,     % gray
                [190, 180, 30]/255,   % olive
                [20, 180, 195]/255,   % cyan
                [90, 165, 255]/255,   % light blue
                [255, 165, 70]/255,   % light orange
                [160, 220, 60]/255,   % light green
                [255, 75, 150]/255    % light red
            };
            
            for i = 1:numDistricts
                if i <= length(palette)
                    colors{i} = palette{i};
                else
                    % Generate more colors if we run out
                    hue = (i-1) / numDistricts;
                    colors{i} = hsv2rgb([hue, 0.85, 0.9]);
                end
            end
        end
        
        % Draw connections between areas
        function drawConnections(obj)
            areas = obj.instance.getAreas();
            
            hold on;
            for i = 1:length(areas)
                area = areas{i};
                id1 = area.getId();
                p1 = obj.areaPositions(id1);
                
                for j = 1:length(area.getNeighbors())
                    neighborId = area.getNeighbors()(j);
                    
                    % Only draw each connection once
                    if neighborId > id1
                        p2 = obj.areaPositions(neighborId);
                        line([p1(1), p2(1)], [p1(2), p2(2)], 'Color', [200, 200, 210, 180]/255, 'LineWidth', 0.9);
                    end
                end
            end
            hold off;
        end
        
        % Draw areas colored by district
        function drawAreas(obj, districtColors)
            areas = obj.instance.getAreas();
            nodeRadius = 9; % Slightly larger nodes
            
            hold on;
            for i = 1:length(areas)
                area = areas{i};
                id = area.getId();
                p = obj.areaPositions(id);
                
                % Find which district this area belongs to
                districtIndex = -1;
                for j = 1:length(obj.zones)
                    if ismember(id, obj.zones{j})
                        districtIndex = j;
                        break;
                    end
                end
                
                % Check if this is a center
                isCenter = false;
                for j = 1:length(obj.centers)
                    if obj.centers(j).getId() == id
                        isCenter = true;
                        break;
                    end
                end
                
                % Draw area node with district color
                if districtIndex >= 1
                    % Fill with district color
                    viscircles(p, nodeRadius, 'Color', districtColors{districtIndex}, ...
                             'EnhanceVisibility', false, 'LineWidth', 0.1);
                    
                    % Draw outline - thicker for centers
                    if isCenter
                        viscircles(p, nodeRadius, 'Color', [40, 40, 40]/255, 'LineWidth', 2, ...
                                 'EnhanceVisibility', false);
                    else
                        viscircles(p, nodeRadius, 'Color', [60, 60, 60]/255, 'LineWidth', 1, ...
                                 'EnhanceVisibility', false);
                        
                        % Draw area ID labels (only for non-centers)
                        text(p(1), p(2) + nodeRadius + 12, num2str(id), ...
                             'Color', [40, 40, 40]/255, 'FontSize', 8, 'HorizontalAlignment', 'center');
                    end
                end
            end
            hold off;
        end
        
        % Draw district centers
        function drawCenters(obj)
            centerRadius = 9; % Match node radius
            
            hold on;
            for i = 1:length(obj.centers)
                center = obj.centers(i);
                id = center.getId();
                p = obj.areaPositions(id);
                
                % Draw cross inside node
                line([p(1) - centerRadius/2, p(1) + centerRadius/2], [p(2), p(2)], ...
                     'Color', [40, 40, 40]/255, 'LineWidth', 2);
                line([p(1), p(1)], [p(2) - centerRadius/2, p(2) + centerRadius/2], ...
                     'Color', [40, 40, 40]/255, 'LineWidth', 2);
                
                % Add center ID label
                text(p(1), p(2) - centerRadius - 5, ['C', num2str(id)], ...
                     'Color', [40, 40, 40]/255, 'FontSize', 9, 'FontWeight', 'bold', ...
                     'HorizontalAlignment', 'center');
            end
            hold off;
        end
        
        % Draw legend
        function drawLegend(obj, districtColors)
            legendX = obj.mainX + obj.mainWidth + 20;
            legendY = 70;
            itemHeight = 25;
            colorBoxSize = 16;
            
            % Draw legend background
            rectangle('Position', [legendX - 10, legendY - 20, obj.width - (obj.mainX + obj.mainWidth) - 30, ...
                                 length(obj.zones) * itemHeight + 120], ...
                     'Curvature', [0.1, 0.1], 'FaceColor', [248, 248, 252]/255, ...
                     'EdgeColor', [220, 220, 230]/255);
            
            % Draw legend title
            text(legendX, legendY, 'Districts', 'Color', [50, 50, 70]/255, ...
                 'FontSize', 12, 'FontWeight', 'bold');
            
            % Draw legend items
            hold on;
            for i = 1:length(obj.zones)
                % Draw color box
                rectangle('Position', [legendX, legendY + 10 + (i-1) * itemHeight, colorBoxSize, colorBoxSize], ...
                         'Curvature', [0.2, 0.2], 'FaceColor', districtColors{i}, ...
                         'EdgeColor', [100, 100, 120]/255);
                
                % Draw district label
                text(legendX + colorBoxSize + 8, legendY + 10 + (i-1) * itemHeight + 12, ...
                     ['District ', num2str(i), ' (C', num2str(obj.centers(i).getId()), ')'], ...
                     'Color', [40, 40, 60]/255, 'FontSize', 10);
            end
            
            % Draw divider line
            line([legendX - 5, legendX + obj.width - (obj.mainX + obj.mainWidth) - 35], ...
                 [legendY + 15 + length(obj.zones) * itemHeight, legendY + 15 + length(obj.zones) * itemHeight], ...
                 'Color', [220, 220, 230]/255);
            
            % Draw node explanation
            explanationY = legendY + 25 + length(obj.zones) * itemHeight;
            text(legendX, explanationY, 'Node Types', 'Color', [50, 50, 70]/255, ...
                 'FontSize', 12, 'FontWeight', 'bold');
            
            % Regular node example
            nodeY = explanationY + 25;
            viscircles([legendX + 8, nodeY], 8, 'Color', [180, 180, 210]/255, 'EnhanceVisibility', false);
            text(legendX + 25, nodeY + 4, 'Regular Area', 'Color', [40, 40, 60]/255, 'FontSize', 10);
            
            % Center node example
            centerY = nodeY + 30;
            viscircles([legendX + 8, centerY], 8, 'Color', [180, 180, 210]/255, 'EnhanceVisibility', false);
            line([legendX + 8 - 4, legendX + 8 + 4], [centerY, centerY], 'Color', [40, 40, 40]/255, 'LineWidth', 2);
            line([legendX + 8, legendX + 8], [centerY - 4, centerY + 4], 'Color', [40, 40, 40]/255, 'LineWidth', 2);
            text(legendX + 25, centerY + 4, 'District Center', 'Color', [40, 40, 60]/255, 'FontSize', 10);
            
            hold off;
        end
    end
end