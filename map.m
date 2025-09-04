map = occupancyMap(10, 10, 1);

% Set all cells to free space (0)
setOccupancy(map, [repmat((1:10)',10,1), kron((1:10)',ones(10,1))], 0);

% Place obstacles
setOccupancy(map, [5, 5], 1);
setOccupancy(map, [6, 6], 1);
setOccupancy(map, [7, 5], 1);

% Define scan angles
scanAngles = linspace(-pi/2, pi/2, 15); % 15 rays from -90° to +90°
maxRange = 5; % Lidar max range

% Define movement path (robot moves from (2,2) to (8,8))
path = [2 2; 3 3; 4 4; 5 5; 6 6; 7 7; 8 8]; 

% Initialize figure
figure;
hold on;
show(map);
title('Robot Moving and Scanning');
grid on;

for i = 1:size(path, 1)
    % Update robot position
    sensorPose = [path(i, 1), path(i, 2), 0];
    
    % Perform ray intersection
    interPts = rayIntersection(map, sensorPose, scanAngles, maxRange);
    
    % Clear previous plot
    cla;
    show(map);
    
    % Plot robot position
    plot(sensorPose(1), sensorPose(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Plot lidar rays
    for j = 1:numel(scanAngles)
        angle = scanAngles(j);
        rayEnd = sensorPose(1:2) + maxRange * [cos(angle), sin(angle)];
        plot([sensorPose(1), rayEnd(1)], [sensorPose(2), rayEnd(2)], 'b--');
    end
    
    % Plot valid intersection points
    validPts = ~isnan(interPts(:, 1));
    plot(interPts(validPts, 1), interPts(validPts, 2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Pause for visualization
    pause(0.5);
end

hold off;