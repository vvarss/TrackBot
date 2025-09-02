clc; clear; close all;

% Get Grid Size
gridSize = input('Enter Grid Size as [rows cols]: ');
% Get Number of Robots
numRobots = input('Enter Number of Robots: ');
% Get Number of Obstacles
numObstacles = input('Enter Number of Obstacles: ');

% Generate Grid
[X, Y] = meshgrid(1:gridSize(2), 1:gridSize(1));

% Plot Grid
figure; hold on;
plot(X, Y, 'k.'); % Plot grid points
xlim([0 gridSize(2)+1]);
ylim([0 gridSize(1)+1]);
grid on;
set(gca, 'XTick', 1:gridSize(2), 'YTick', 1:gridSize(1), 'GridAlpha', 0.3);
xlabel('X'); ylabel('Y'); title('Ordinary Grid');
hold off;

disp("Generating grid...");

% Generate random obstacles
obstacles = randi([1 gridSize(1)], numObstacles, 2);

% Generate random robot positions
robotPositions = randi([1 gridSize(1)], numRobots, 2);

% Set a goal position
goalPos = randi([1 gridSize(1)], 1, 2);

% Q-Learning Parameters
gamma = 0.95; 
alpha = 0.1; 
epsilon = 0.1; 
episodes = 500;

actions = [0 1; 0 -1; -1 0; 1 0];  % Right, Left, Up, Down
Q_table = zeros(gridSize(1), gridSize(2), 4);
pathLengths = zeros(numRobots, 1);

% Training Loop
tic;
for ep = 1:episodes
    for r = 1:numRobots
        pos = robotPositions(r, :);
        steps = 0;
        while ~isequal(pos, goalPos) && steps < (gridSize(1) * gridSize(2))
            steps = steps + 1;
            if rand < epsilon
                actionIdx = randi(4);
            else
                [~, actionIdx] = max(Q_table(pos(1), pos(2), :));
            end
            newPos = max(min(pos + actions(actionIdx, :), gridSize), [1,1]);
            if isequal(newPos, goalPos)
                reward = 100;
            elseif ismember(newPos, obstacles, 'rows')
                reward = -10;
            else
                reward = -0.1;
            end
            Q_table(pos(1), pos(2), actionIdx) = ...
                (1 - alpha) * Q_table(pos(1), pos(2), actionIdx) + ...
                alpha * (reward + gamma * max(Q_table(newPos(1), newPos(2), :)));
            pos = newPos;
        end
        pathLengths(r) = steps;
    end
end
computationTime = toc;
disp("Plotting...");

% Visualization
figure; hold on;
axis([1 gridSize(1) 1 gridSize(2)]);
grid on;
axis equal;
set(gca, 'XTick', 1:gridSize(1), 'YTick', 1:gridSize(2), 'GridColor', 'k');
set(gca, 'YDir', 'normal');
xlabel('X'); ylabel('Y'); title('Multi-Robot Path Planning');

% Plot Grid
plot(obstacles(:,1), obstacles(:,2), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPos(1), goalPos(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2);

% Plot Robots
robotPlots = gobjects(numRobots, 1);
for r = 1:numRobots
    robotPlots(r) = plot(robotPositions(r,1), robotPositions(r,2), ...
        'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
end

% Simulate Movement
collisionCount = 0;
robotPaths = cell(numRobots,1);
for r = 1:numRobots
    pos = robotPositions(r, :);
    path = pos;
    while ~isequal(pos, goalPos) && size(path,1) < (gridSize(1) * gridSize(2))
        [~, actionIdx] = max(Q_table(pos(1), pos(2), :));
        newPos = max(min(pos + actions(actionIdx, :), gridSize), [1,1]);
        if ismember(newPos, obstacles, 'rows')
            collisionCount = collisionCount + 1;
            break;
        else
            pos = newPos;
            path = [path; pos];
        end
    end
    robotPaths{r} = path;
end

% Metrics Calculation
truePositives = sum(pathLengths < (gridSize(1) * gridSize(2))); 
falsePositives = collisionCount; 
falseNegatives = numRobots - truePositives;
trueNegatives = numRobots - collisionCount;

precision = truePositives / numRobots;
accuracy = (truePositives) / numRobots;
f1_score = 2 * (precision * accuracy) / (precision + accuracy);
collisionAvoidanceRate = 1 - (collisionCount / numRobots);

% Print Metrics
fprintf('F1 Score: %.4f\n', f1_score);
fprintf('Accuracy: %.4f\n', accuracy);
fprintf('Precision: %.4f\n', precision);
fprintf('Collision Avoidance Rate: %.4f\n', collisionAvoidanceRate);
fprintf('Computation Time: %.4f seconds\n', computationTime);

% Animate Movement
maxSteps = max(cellfun(@(x) size(x,1), robotPaths));
trajPlots = gobjects(numRobots, 1);
for r = 1:numRobots
    trajPlots(r) = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
end
for t = 1:maxSteps
    for r = 1:numRobots
        if t <= size(robotPaths{r},1)
            set(robotPlots(r), 'XData', robotPaths{r}(t,1), 'YData', robotPaths{r}(t,2));
            set(trajPlots(r), 'XData', robotPaths{r}(1:t,1), 'YData', robotPaths{r}(1:t,2));
        end
    end
    pause(0.5);
end
hold off;
