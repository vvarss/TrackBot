%{
function newState = robotDynamics(state, action)
    % Define possible movement actions: [Right, Left, Up, Down]
    actions = [1, 0; -1, 0; 0, 1; 0, -1];

    % Define warehouse grid size (Example: 10x10)
    gridSize = [10, 10]; % [Rows, Columns]

    % Default initial state if not provided
    if nargin < 2
        disp("No input provided. Initializing default state and action.")
        state = [1, 1]; % Starting position in the warehouse
        action = 1; % Default action (Move Right)
    elseif nargin < 1
        error("robotDynamics requires at least the 'state' input.");
    end

    % Ensure action is within bounds (1 to 4)
    if action < 1 || action > 4
        error("Invalid action. Must be between 1 (Right) and 4 (Down).");
    end

    % Compute new position
    newState = state + actions(action, :);

    % Ensure newState is within warehouse boundaries
    if any(newState < 1) || any(newState > gridSize)
        disp("Move blocked: Out of bounds! Staying in the same position.")
        newState = state; % Revert to previous state if out of bounds
    end

    % Display the movement
    fprintf("State changed from [%d, %d] to [%d, %d] using action %d\n", ...
            state(1), state(2), newState(1), newState(2), action);
end

%}

%{
function [nextState, newPos] = robotDynamics(state, action, robotPos)
    % Robot Dynamics: Update state and position based on action
    % Inputs:
    %   state - Current state index
    %   action - Action selected by policy
    %   robotPos - Current [x, y] position
    % Outputs:
    %   nextState - Updated state index
    %   newPos - Updated [x, y] position
    
    % Default values for automatic execution
    if nargin < 1 || isempty(state)
        state = 10;
    end
    if nargin < 2 || isempty(action)
        action = 'Move Forward';
    end
    if nargin < 3 || isempty(robotPos)
        robotPos = [50, 50]; % Default starting position
    end
    
    % Define motion step size
    stepSize = 5;
    
    % Update position based on action
    switch action
        case 'Move Forward'
            newPos = [robotPos(1), min(robotPos(2) + stepSize, 100)];
        case 'Move Reverse'
            newPos = [robotPos(1), max(robotPos(2) - stepSize, 0)];
        case 'Turn Left'
            newPos = [max(robotPos(1) - stepSize, 0), robotPos(2)];
        case 'Turn Right'
            newPos = [min(robotPos(1) + stepSize, 100), robotPos(2)];
        otherwise
            newPos = robotPos; % No movement if invalid action
    end
    
    % Compute new state based on position
    nextState = round(newPos(1) / 10) * 10 + round(newPos(2) / 10);
    
    % Automatically execute if script runs independently
    if nargout == 0
        disp(['Next State: ', num2str(nextState)]);
        disp(['New Position: [', num2str(newPos(1)), ', ', num2str(newPos(2)), ']']);
    end
end
%}

%{
function [nextState, newPos, stuckCounter] = robotDynamics(state, action, robotPos, stuckCounter)
    newPos = robotPos; % Default to current position
    
    % Define movement based on action
    switch action
        case 1 % Move Up
            newPos(2) = min(newPos(2) + 1, 10);
        case 2 % Move Down
            newPos(2) = max(newPos(2) - 1, 1);
        case 3 % Move Left
            newPos(1) = max(newPos(1) - 1, 1);
        case 4 % Move Right
            newPos(1) = min(newPos(1) + 1, 10);
    end

    % Debugging
    fprintf('DEBUG: Action = %d\n', action);
    fprintf('DEBUG: Previous Position = [%d %d]\n', robotPos(1), robotPos(2));
    fprintf('DEBUG: New Position = [%d %d]\n', newPos(1), newPos(2));

    % Check if the robot is stuck
    if isequal(newPos, robotPos)
        stuckCounter = stuckCounter + 1;
        if stuckCounter > 5
            disp('WARNING: Robot is stuck! Taking random action.');
            newPos = [randi(10), randi(10)]; % Randomly reposition
            stuckCounter = 0;
        end
    else
        stuckCounter = 0;
    end
    
    nextState = newPos;
end
%}


%{
function [nextState, newPos, stuckCounter] = robotDynamics(state, action, robotPos, stuckCounter)
    % Provide default values if arguments are missing
    if nargin < 4
        stuckCounter = 0;
    end
    if nargin < 3
        robotPos = [1, 1]; % Default starting position
    end
    if nargin < 2
        action = 1; % Default action (Move Up)
    end
    if nargin < 1
        state = robotPos; % Default state
    end

    newPos = robotPos; % Default to current position
    
    % Define movement based on action
    switch action
        case 1 % Move Up
            newPos(2) = min(newPos(2) + 1, 10);
        case 2 % Move Down
            newPos(2) = max(newPos(2) - 1, 1);
        case 3 % Move Left
            newPos(1) = max(newPos(1) - 1, 1);
        case 4 % Move Right
            newPos(1) = min(newPos(1) + 1, 10);
    end

    % Debugging
    fprintf('DEBUG: Action = %d\n', action);
    fprintf('DEBUG: Previous Position = [%d %d]\n', robotPos(1), robotPos(2));
    fprintf('DEBUG: New Position = [%d %d]\n', newPos(1), newPos(2));

    % Check if the robot is stuck
    if isequal(newPos, robotPos)
        stuckCounter = stuckCounter + 1;
        if stuckCounter > 5
            disp('WARNING: Robot is stuck! Taking random action.');
            newPos = [randi(10), randi(10)]; % Randomly reposition
            stuckCounter = 0;
        end
    else
        stuckCounter = 0;
    end
    
    nextState = newPos;
end
%}

%{
function data = robotDynamics(input)
    data = zeros(2,1);  % Initialize to prevent undefined output

    if input > 0
        data = [1; 0];  % Ensure all paths assign a value
    elseif input < 0
        data = [0; 1];  % Define for all cases
    end
end
%}

%{
function [nextState, newPos, stuckCounter] = robotDynamics(state, action, robotPos, stuckCounter)
    % Ensure all outputs are always defined
    nextState = [0, 0]; % Default value
    newPos = [0, 0];    % Default value
    if nargin < 4, stuckCounter = 0; end
    if nargin < 3, robotPos = [1, 1]; end
    if nargin < 2, action = 1; end
    if nargin < 1, state = robotPos; end

    newPos = robotPos; % Default to current position

    % Define movement based on action
    switch action
        case 1 % Move Up
            newPos(2) = min(newPos(2) + 1, 10);
        case 2 % Move Down
            newPos(2) = max(newPos(2) - 1, 1);
        case 3 % Move Left
            newPos(1) = max(newPos(1) - 1, 1);
        case 4 % Move Right
            newPos(1) = min(newPos(1) + 1, 10);
    end

    % Ensure `nextState` is properly assigned
    nextState = newPos;

    % Check if the robot is stuck
    if isequal(newPos, robotPos)
        stuckCounter = stuckCounter + 1;
        if stuckCounter > 5
            newPos = [randi(10), randi(10)]; % Random reposition
            stuckCounter = 0;
        end
    else
        stuckCounter = 0;
    end
end
%}
%{
function data = robotDynamics(input)
    % Initialize output variable
    data = zeros(2, 1);  % Example default size (2x1), adjust as needed

    % Your logic here
    if input > 0
        data = [1; 2];  % Example assignment
    elseif input < 0
        data = [-1; -2];  % Another example
    end
end
%}

clc; clear; close all;

%% PARAMETERS
grid_size = 20;
num_robots = 5;
num_obstacles = 15;
goal_position = [grid_size/2, grid_size/2]; % Single goal
max_episodes = 10;
max_steps = 200;
learning_rate = 0.01;
discount_factor = 0.95;
batch_size = 32;
epsilon = 1.0;
epsilon_decay = 0.995;
min_epsilon = 0.01;

collision_count = 0;
obstacles_avoided = 0;
training_time = 0;
path_lengths = zeros(num_robots, max_episodes);
computation_times = zeros(num_robots, max_episodes);

% Initialize Q-tables
Q_table = rand(grid_size, grid_size, 4);
Q_target = Q_table;

% Generate random obstacles
obstacles = randi(grid_size, num_obstacles, 2);

% Initialize robot positions
robot_positions = randi(grid_size, num_robots, 2);

% Visualization setup
figure;
hold on;
axis([0 grid_size 0 grid_size]);
grid on;

% Draw obstacles
for i = 1:num_obstacles
    plot(obstacles(i,1), obstacles(i,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end

% Draw goal
plot(goal_position(1), goal_position(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');

% Initialize robot markers
robot_markers = gobjects(num_robots, 1);
for i = 1:num_robots
    robot_markers(i) = plot(robot_positions(i,1), robot_positions(i,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
end

%% TRAINING LOOP
tic;
for episode = 1:max_episodes
    fprintf('Episode %d/%d\n', episode, max_episodes);
    for robot = 1:num_robots
        position = robot_positions(robot, :);
        path_length = 0;
        done = false;

        for step = 1:max_steps
            if rand < epsilon
                action = randi(4);
            else
                [~, action] = max(Q_table(position(1), position(2), :));
            end

            next_position = get_next_position(position, action, grid_size);

            if any(ismember(obstacles, next_position, 'rows'))
                reward = -1;
                collision_count = collision_count + 1;
                break;
            else
                reward = -0.1;
                obstacles_avoided = obstacles_avoided + 1;
            end

            if isequal(next_position, goal_position)
                reward = 10;
                done = true;
            end

            % Update Q-table
            Q_table = update_ddqn(Q_table, Q_target, position, action, next_position, reward, learning_rate, discount_factor);

            position = next_position;
            path_length = path_length + 1;

            % Update robot position
            if isvalid(robot_markers(robot))
                set(robot_markers(robot), 'XData', position(1), 'YData', position(2));
                drawnow;
            end

            pause(0.05);

            if done, break; end
        end

        path_lengths(robot, episode) = path_length;
        computation_times(robot, episode) = toc;
    end

    if epsilon > min_epsilon
        epsilon = epsilon * epsilon_decay;
    end

    if mod(episode, 2) == 0
        Q_target = Q_table;
    end
end
training_time = toc;

%% PERFORMANCE METRICS
evaluate_performance(path_lengths, computation_times, max_episodes, collision_count, obstacles_avoided, training_time, learning_rate, discount_factor, batch_size, Q_table);

%% FUNCTION: Get Next Position
function next_position = get_next_position(position, action, grid_size)
    next_position = position;
    if action == 1 && position(2) < grid_size, next_position(2) = position(2) + 1; end
    if action == 2 && position(2) > 1, next_position(2) = position(2) - 1; end
    if action == 3 && position(1) > 1, next_position(1) = position(1) - 1; end
    if action == 4 && position(1) < grid_size, next_position(1) = position(1) + 1; end
end

%% FUNCTION: Update DDQN
function Q_table = update_ddqn(Q_table, Q_target, position, action, next_position, reward, learning_rate, discount_factor)
    [~, best_action] = max(Q_table(next_position(1), next_position(2), :));
    target_q = Q_target(next_position(1), next_position(2), best_action);
    Q_table(position(1), position(2), action) = ...
        (1 - learning_rate) * Q_table(position(1), position(2), action) ...
        + learning_rate * (reward + discount_factor * target_q);
end

%% FUNCTION: Evaluate Performance
function evaluate_performance(path_lengths, computation_times, max_episodes, collision_count, obstacles_avoided, training_time, learning_rate, discount_factor, batch_size, Q_table)
    num_robots = size(path_lengths, 1);
    total_trials = max_episodes * num_robots;
    successful_paths = sum(path_lengths(:) > 0);

    accuracy = successful_paths / total_trials;
    precision = successful_paths / (successful_paths + collision_count);
    f1_score = 2 * (precision * accuracy) / (precision + accuracy);

    actual_paths = path_lengths(path_lengths > 0);
    predicted_paths = mean(actual_paths) * ones(size(actual_paths));
    rmse = sqrt(mean((actual_paths - predicted_paths).^2));

    scalability = num_robots / (collision_count + 1);

    model_size = whos('Q_table');
    model_size_MB = model_size.bytes / 1e6;

    fprintf('\n--- PERFORMANCE METRICS ---\n');
    fprintf('Total Robots: %d\n', num_robots);
    fprintf('Total Episodes: %d\n', max_episodes);
    fprintf('Training Time: %.2f sec\n', training_time);
    fprintf('Collisions: %d\n', collision_count);
    fprintf('Obstacles Avoided: %d\n', obstacles_avoided);
    fprintf('Scalability: %.4f\n', scalability);
    fprintf('Accuracy: %.4f\n', accuracy);
    fprintf('Precision: %.4f\n', precision);
    fprintf('F1 Score: %.4f\n', f1_score);
    fprintf('Model Size: %.2f MB\n', model_size_MB);
end
