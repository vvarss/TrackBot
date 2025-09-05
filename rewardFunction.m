%{
function isObstacle = detectObstacle(state)
    % Detects obstacles in the warehouse environment based on the state index
    % Inputs:
    %   state - Current state index
    % Outputs:
    %   isObstacle - True if obstacle detected, False otherwise
    
    % Default state if not provided
    if nargin < 1 || isempty(state)
        state = 10; % Default test state
    end
    
    % Define fixed obstacle locations for consistency
    obstacleStates = [15, 23, 32, 42, 50, 56, 63, 72, 78, 85, 92];
    
    % Check if current state is an obstacle
    isObstacle = any(state == obstacleStates);
    
    % Debugging: Display obstacle detection status
    fprintf('Checking State: %d\n', state);
    fprintf('Obstacle States: %s\n', mat2str(obstacleStates));
    fprintf('Obstacle Detected: %d\n', isObstacle);
end
%}

%{
function reward = computeReward(state, goalPos)
    % Compute reward for Q-learning based on obstacle detection & goal proximity
    % Inputs:
    %   state - Current state index
    %   goalPos - Target goal position [x, y]
    % Output:
    %   reward - Reward value
    
    % Default values if missing
    if nargin < 1 || isempty(state)
        state = 10; % Default state
    end
    if nargin < 2 || isempty(goalPos)
        goalPos = [90, 90]; % Default goal position
    end
    
    % Convert state to position (assuming 10x10 grid mapping)
    % Convert state to position (assuming 10x10 grid mapping)
    posX = mod(state - 1, env.GridSize(1)) + 1; % Convert state index to X coordinate
    posY = floor((state - 1) / env.GridSize(1)) + 1; % Convert state index to Y coordinate

% Ensure goalPos is correctly formatted
    goalX = goalPos(1);
    goalY = goalPos(2);

% Compute distance properly
distanceToGoal = norm([posX, posY] - [goalX, goalY]); % Ensures correct vector math

    % Define base rewards
    goalReward = 1000; % High reward for reaching goal
    collisionPenalty = -500; % Heavy penalty for collision
    stepPenalty = -1; % Small penalty to encourage shorter paths
    
    % Check if the robot is near the goal
    if numel(goalPos) ~= 2
      error('goalPos must be in [x, y] format. Current goalPos: %s', mat2str(goalPos));
    end
    disp(['DEBUG: goalPos = ', mat2str(goalPos)]);
    disp(['DEBUG: posX = ', num2str(posX), ', posY = ', num2str(posY)]);
    disp(['DEBUG: goalPos size = ', mat2str(size(goalPos))]);

    distanceToGoal = norm([posX, posY] - goalPos(1:2)); % Ensure goalPos is [x, y]

    if distanceToGoal < 5
        reward = goalReward;
        return;
    end
    
    % Check for obstacles (using external sensor processing function)
    if exist('detectObstacle.m', 'file') && detectObstacle(state)
        reward = collisionPenalty;
    else
        reward = stepPenalty - (distanceToGoal / 100); % Encourages moving toward goal
    end
    
    % Automatically display reward if function runs independently
    if nargout == 0
        fprintf('State: %d, Position: [%d, %d]\n', state, posX, posY);
        fprintf('Computed Reward: %d\n', reward);
    end
end
%}


%{
function reward = rewardFunction(state, goalPos, gridSize)
    % Set default values if no inputs are provided (for testing)
    if nargin < 1 || isempty(state)
        state = 10; % Default state index
    end
    if nargin < 2 || isempty(goalPos)
        goalPos = [10, 10]; % Default goal position
    end
    if nargin < 3 || isempty(gridSize)
        gridSize = [10, 10]; % Default warehouse size
    end

    % Convert state to position (assuming grid-based mapping)
    posX = mod(state - 1, gridSize(1)) + 1; % X coordinate
    posY = floor((state - 1) / gridSize(1)) + 1; % Y coordinate

    % Ensure goalPos is correctly formatted
    goalX = goalPos(1);
    goalY = goalPos(2);

    % Compute distance properly
    %distanceToGoal = norm([posX, posY] - [goalX, goalY]); % Fixes size mismatch
    % Ensure posX and posY are scalars
posX = mod(state - 1, gridSize(1)) + 1;
posY = floor((state - 1) / gridSize(1)) + 1;

% Ensure goalX and goalY are scalars
goalX = goalPos(1);
goalY = goalPos(2);

% Debugging
disp(['DEBUG: posX = ', num2str(posX), ', posY = ', num2str(posY)]);
disp(['DEBUG: goalX = ', num2str(goalX), ', goalY = ', num2str(goalY)]);

% Ensure everything is scalar before computing distance
if numel(posX) > 1 || numel(posY) > 1 || numel(goalX) > 1 || numel(goalY) > 1
    error('Mismatch in variable sizes! Check posX, posY, goalX, goalY.');
end

% Compute distance properly
distanceToGoal = norm([posX, posY] - [goalX, goalY]); % Ensures correct vector math

    % Define rewards
    goalReward = 1000;
    collisionPenalty = -500;
    stepPenalty = -1;

    % Check if robot reached the goal
    if distanceToGoal < 2
        reward = goalReward;
    if exist('detectObstacle.m', 'file')
     isObstacle = detectObstacle(state);
    else
    warning('detectObstacle.m not found! Assuming no obstacles.');
    isObstacle = false;
end

if distanceToGoal < 2
    reward = goalReward;
elseif isObstacle
    reward = collisionPenalty;
else
    reward = stepPenalty - (distanceToGoal / 100);
end

    else
        reward = stepPenalty - (distanceToGoal / 100);
    end

    % Debugging Output
    disp(['DEBUG: state = ', num2str(state)]);
    disp(['DEBUG: posX = ', num2str(posX), ', posY = ', num2str(posY)]);
    disp(['DEBUG: goalPos = ', mat2str(goalPos)]);
    disp(['DEBUG: reward = ', num2str(reward)]);
end
%}

%{
function reward = rewardFunction(state, goalPos, gridSize)
    % Set default values if no inputs are provided (for standalone testing
    if nargin < 1 || isempty(state)
        state = [1, 1]; % Default state as [x, y]
    end
    if nargin < 2 || isempty(goalPos)
        goalPos = [10, 10]; % Default goal position
    end
    if nargin < 3 || isempty(gridSize)
        gridSize = [10, 10]; % Default warehouse grid size
    end

    % Ensure input formats are correct
    if numel(state) ~= 2
        error('State must be in [x, y] format. Received: %s', mat2str(state));
    end
    if numel(goalPos) ~= 2
        error('goalPos must be in [x, y] format. Received: %s', mat2str(goalPos));
    end

    % Extract X and Y positions correctly
    posX = state(1);
    posY = state(2);
    goalX = goalPos(1);
    goalY = goalPos(2);

    % Compute correct Euclidean distance
    distanceToGoal = sqrt((posX - goalX)^2 + (posY - goalY)^2);

    % Define rewards
    goalReward = 1000;
    collisionPenalty = -500;
    stepPenalty = -1;

    % Determine reward
    if distanceToGoal < 2
        reward = goalReward;
    elseif exist('detectObstacle.m', 'file') && detectObstacle(state)
        reward = collisionPenalty;
    else
        reward = stepPenalty - (distanceToGoal / 10); % Encourage movement
    end

    % Debugging Output
    disp(['DEBUG: State = ', mat2str(state)]);
    disp(['DEBUG: Goal Position = ', mat2str(goalPos)]);
    disp(['DEBUG: Distance to Goal = ', num2str(distanceToGoal)]);
    disp(['DEBUG: Reward = ', num2str(reward)]);
end
%}
function reward = rewardFunction(state, goalPos, gridSize)
    % Ensure correct input format
    if numel(state) ~= 2 || numel(goalPos) ~= 2
        error('State and goalPos must be in [x, y] format.');
    end

    % Extract positions
    posX = state(1);
    posY = state(2);
    goalX = goalPos(1);
    goalY = goalPos(2);

    % Compute Euclidean distance
    distanceToGoal = sqrt((posX - goalX)^2 + (posY - goalY)^2);

    % Define rewards
    goalReward = 1000;
    collisionPenalty = -500;
    movementPenalty = -2;  % Encourage movement
    stepReward = -distanceToGoal * 2; % Stronger incentive to move toward goal

    % Determine reward
    if distanceToGoal < 2
        reward = goalReward;
    elseif exist('detectObstacle.m', 'file') && detectObstacle(state)
        reward = collisionPenalty;
    else
        reward = stepReward + movementPenalty;
    end
end
