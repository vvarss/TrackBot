%{
function trainAMRAgent()
    %% ✅ Define Warehouse Environment (10x10 Grid)
    gridSize = [10, 10]; % Define warehouse size (10x10)
    startPos = [1, 1];   % AMR starting position
    goalPos = [10, 10];  % Goal position in warehouse

    % Define warehouse obstacles (1 = obstacle, 0 = free space)
    envGrid = zeros(gridSize);
    envGrid(3:8, 5) = 1;  % Vertical shelf in column 5
    envGrid(6, 2:6) = 1;  % Horizontal shelf row

    % Convert to binary occupancy map
    map = binaryOccupancyMap(envGrid);

    %% ✅ Define Action Space (Up, Down, Left, Right)
    actions = [-1, 0; 1, 0; 0, -1; 0, 1]; % Up, Down, Left, Right
    numActions = size(actions, 1);

    %% ✅ Q-Learning Parameters (for Reinforcement Learning)
    maxEpisodes = 500;   % Number of training episodes
    alpha = 0.1;         % Learning rate
    gamma = 0.95;        % Discount factor
    epsilon = 1.0;       % Initial exploration rate
    epsilonDecay = 0.99; % Decay factor for exploration

    % Initialize Q-table with zeros
    Q = zeros(gridSize(1), gridSize(2), numActions);

    %% ✅ Training Loop
    for episode = 1:maxEpisodes
        state = startPos;
        episodeReward = 0;

        while ~isequal(state, goalPos)
            % Epsilon-Greedy Action Selection
            if rand < epsilon
                actionIdx = randi(numActions); % Random action (explore)
            else
                [~, actionIdx] = max(Q(state(1), state(2), :)); % Greedy action (exploit)
            end

            % Take action and move
            newState = state + actions(actionIdx, :);

            % Check if move is valid (boundary & obstacle check)
            if newState(1) < 1 || newState(1) > gridSize(1) || newState(2) < 1 || newState(2) > gridSize(2) || envGrid(newState(1), newState(2)) == 1
                reward = -10; % Penalize hitting obstacles or going out of bounds
                newState = state; % Stay in place
            elseif isequal(newState, goalPos)
                reward = 100; % Big reward for reaching goal
            else
                reward = -1; % Step penalty to encourage shortest path
            end

            % Q-learning update using Bellman Equation
            maxNextQ = max(Q(newState(1), newState(2), :)); % Max future Q-value
            Q(state(1), state(2), actionIdx) = (1 - alpha) * Q(state(1), state(2), actionIdx) ...
                                             + alpha * (reward + gamma * maxNextQ);

            % Move to new state
            state = newState;
            episodeReward = episodeReward + reward;
        end

        % Decay epsilon to reduce exploration over time
        epsilon = max(0.1, epsilon * epsilonDecay);

        % Display progress every 100 episodes
        if mod(episode, 100) == 0
            fprintf('Episode %d | Total Reward: %d | Epsilon: %.3f\n', episode, episodeReward, epsilon);
        end
    end

    disp('Training Complete!');

    %% ✅ Extract the final learned path
    state = startPos;
    path = state;
    while ~isequal(state, goalPos)
        [~, actionIdx] = max(Q(state(1), state(2), :)); % Choose best action
        newState = state + actions(actionIdx, :);

        if newState(1) < 1 || newState(1) > gridSize(1) || newState(2) < 1 || newState(2) > gridSize(2) || envGrid(newState(1), newState(2)) == 1
            break; % Stop if invalid move
        end

        path = [path; newState];
        state = newState;
    end

    %% ✅ Visualization
    figure;
    show(map);
    hold on;
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2); % Plot learned path
    plot(startPos(1), startPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start (Green)
    plot(goalPos(1), goalPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Goal (Red)
    hold off;
    title('Q-Learning Based AMR Path Planning');
end

% Run the function automatically
trainAMRAgent();
%}

function trainAgent()
    % Grid and Environment Setup
    gridSize = [10, 10]; % Grid dimensions
    startPos = [3, 1];   % Start position
    goalPos = [5, 9];    % Goal position

    % Define obstacles in the environment
    envGrid = zeros(gridSize); % 0 = Free, 1 = Obstacle
    envGrid(2:8, 5) = 1;       % Vertical obstacles in column 5

    % Q-Learning Parameters
    alpha = 0.1;    % Learning rate
    gamma = 0.9;    % Discount factor
    epsilon = 0.1;  % Exploration rate
    numEpisodes = 1000;  % Training iterations

    % Q-table Initialization
    Q = zeros(gridSize(1), gridSize(2), 4); % 4 actions per state

    % Action Mapping (Up, Down, Left, Right)
    actions = [-1, 0; 1, 0; 0, -1; 0, 1];

    % Training Loop
    for episode = 1:numEpisodes
        % Reset agent to start position
        state = startPos;
        
        while ~isequal(state, goalPos)
            % Epsilon-Greedy Action Selection
            if rand < epsilon
                actionIdx = randi(4); % Random action (explore)
            else
                [~, actionIdx] = max(Q(state(1), state(2), :)); % Greedy action (exploit)
            end
            
            % Take action and move
            newState = state + actions(actionIdx, :);
            
            % Check if move is valid
            if newState(1) < 1 || newState(1) > gridSize(1) || newState(2) < 1 || newState(2) > gridSize(2)
                reward = -10; % Penalize hitting walls
                newState = state; % Stay in place
            elseif envGrid(newState(1), newState(2)) == 1
                reward = -10; % Penalize hitting obstacles
                newState = state; % Stay in place
            elseif isequal(newState, goalPos)
                reward = 10; % Reward for reaching goal
            else
                reward = -1; % Penalize step to encourage shortest path
            end
            
            % Update Q-table
            Q(state(1), state(2), actionIdx) = (1 - alpha) * Q(state(1), state(2), actionIdx) ...
                                              + alpha * (reward + gamma * max(Q(newState(1), newState(2), :)));
            
            % Move to new state
            state = newState;
        end
    end

    disp('Training Complete!');

    % Path Extraction for Visualization
    state = startPos;
    path = state; % Store path
    while ~isequal(state, goalPos)
        [~, actionIdx] = max(Q(state(1), state(2), :)); % Best action
        newState = state + actions(actionIdx, :);
        
        % Ensure the agent does not loop in an invalid state
        if newState(1) < 1 || newState(1) > gridSize(1) || newState(2) < 1 || newState(2) > gridSize(2) || envGrid(newState(1), newState(2)) == 1
            break; % Stop if the agent is stuck
        end
        
        path = [path; newState]; % Append new state to path
        state = newState;
    end

    % Display Final Path
    figure;
    imagesc(envGrid'); % Flip matrix for proper visualization
    colormap(gray); % 0 = white (free), 1 = black (obstacle)
    hold on;
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2); % Plot learned path
    plot(startPos(1), startPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start (Green)
    plot(goalPos(1), goalPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Goal (Red)
    hold off;
    title('Q-Learning Path Planning');
end

% Run the function automatically
trainAgent();
