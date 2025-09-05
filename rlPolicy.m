%{
function policyNet = rlPolicy(stateDim, numActions)
    % ✅ Define Inputs
    stateInput = featureInputLayer(stateDim, 'Normalization', 'none', 'Name', 'state');
    actionInput = featureInputLayer(1, 'Normalization', 'none', 'Name', 'action'); % Action Input

    % ✅ Define Hidden Layers
    layers = [
        concatenationLayer(1, 2, 'Name', 'concat')
        fullyConnectedLayer(64, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(64, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(1, 'Name', 'output') % Q-value output for a single action
    ];

    % ✅ Create Layer Graph
    lgraph = layerGraph(stateInput);
    lgraph = addLayers(lgraph, actionInput);
    lgraph = addLayers(lgraph, layers);

    % ✅ Connect Inputs to Concatenation Layer
    lgraph = connectLayers(lgraph, 'state', 'concat/in1');
    lgraph = connectLayers(lgraph, 'action', 'concat/in2');

    % ✅ Convert to `dlnetwork`
    policyNet = dlnetwork(lgraph);

    % ✅ Display Confirmation
    disp("✅ RL Policy Created with " + num2str(numActions) + " Actions!");
end
%}
%{
function [Q, policy] = rlPolicy(Q, state, actionSet, reward, nextState, alpha, gamma, epsilon, miniBatch, mlpModel)
    % Q-Learning Policy Function with MLP-NN
    % Ensure all inputs have default values
    stateSpace = 100; % Define the total number of states
    numActions = 4; % Define number of possible actions
    
    if nargin < 1 || isempty(Q)
        Q = rand(stateSpace, numActions) * 0.01; % Small random values to allow learning
    end
    if nargin < 2 || isempty(state) || state < 1 || state > stateSpace
        state = randi(stateSpace); % Assign a valid random state
    end
    if nargin < 3 || isempty(actionSet)
        actionSet = {'Move Forward', 'Move Reverse', 'Turn Left', 'Turn Right'};
    end
    if nargin < 4 || isempty(reward)
        reward = 0; % Default reward value
    end
    if nargin < 5 || isempty(nextState) || nextState < 1 || nextState > stateSpace
        nextState = randi(stateSpace); % Assign a valid random next state
    end
    if nargin < 6 || isempty(alpha)
        alpha = 0.1; % Default learning rate
    end
    if nargin < 7 || isempty(gamma)
        gamma = 0.9; % Default discount factor
    end
    if nargin < 8 || isempty(epsilon)
        epsilon = 0.1; % Default exploration rate
    end
    if nargin < 9 || isempty(miniBatch)
        miniBatch = 64; % Default mini-batch size
    end
    if nargin < 10 || isempty(mlpModel)
        mlpModel = []; % Placeholder for MLP model (needs to be trained)
    end
    
    % Ensure Q-table has correct dimensions
    if size(Q,1) < stateSpace || size(Q,2) < numActions
        Q = rand(stateSpace, numActions) * 0.01; % Ensure Q-table is not all zeros
    end
    
    % Ensure state is a valid index
    state = max(1, min(state, stateSpace));
    nextState = max(1, min(nextState, stateSpace));
    
    % Select action using epsilon-greedy policy
    if rand < epsilon
        actionIdx = randi(numActions); % Random action (Exploration)
    else
        qInputs = [state, Q(state, :)]; % Input to MLP (state + current Q-values)
        if ~isempty(mlpModel)
            Q_values = predict(mlpModel, qInputs); % Predict Q-values using MLP
        else
            Q_values = Q(state, :); % Use Q-table if MLP is not available
        end
        [~, actionIdx] = max(Q_values); % Best action (Exploitation)
    end
    action = actionSet{actionIdx};
    
    % Q-learning Update Rule with MLP-NN
    if ~isempty(mlpModel)
        maxNextQ = max(predict(mlpModel, [nextState, Q(nextState, :)]));
    else
        maxNextQ = max(Q(nextState, :));
    end
    targetQ = reward + gamma * maxNextQ;
    Q(state, actionIdx) = Q(state, actionIdx) + alpha * (targetQ - Q(state, actionIdx));
    
    % Mini-batch Training with MLP
    if ~isempty(mlpModel)
        batchStates = randi(size(Q, 1), miniBatch, 1); % Sample random states for training
        batchInputs = [batchStates, Q(batchStates, :)];
        batchTargets = reward + gamma * max(predict(mlpModel, batchInputs), [], 2);
        train(mlpModel, batchInputs, batchTargets); % Train MLP with mini-batch
    end
    
    % Return updated policy
    [~, bestActionIdx] = max(Q(state, :));
    policy = actionSet{bestActionIdx};
end
%}
function [Q, policy] = rlPolicy(Q, state, actionSet, reward, nextState, alpha, gamma, epsilon, miniBatch, mlpModel)
    % Q-Learning Policy Function with MLP-NN and Obstacle Detection
    % Ensure all inputs have default values
    stateSpace = 100; % Define the total number of states
    numActions = 4; % Define number of possible actions
    
    if nargin < 1 || isempty(Q)
        Q = rand(stateSpace, numActions) * 0.01; % Small random values to allow learning
    end
    if nargin < 2 || isempty(state) || state < 1 || state > stateSpace
        state = randi(stateSpace); % Assign a valid random state
    end
    if nargin < 3 || isempty(actionSet)
        actionSet = {'Move Forward', 'Move Reverse', 'Turn Left', 'Turn Right'};
    end
    if nargin < 4 || isempty(reward)
        reward = computeReward(state); % Use correct reward function
    end
    if nargin < 5 || isempty(nextState) || nextState < 1 || nextState > stateSpace
        nextState = getNextState(state, actionSet); % Get next state from robot dynamics
    end
    if nargin < 6 || isempty(alpha)
        alpha = 0.1; % Default learning rate
    end
    if nargin < 7 || isempty(gamma)
        gamma = 0.9; % Default discount factor
    end
    if nargin < 8 || isempty(epsilon)
        epsilon = 0.1; % Default exploration rate
    end
    if nargin < 9 || isempty(miniBatch)
        miniBatch = 64; % Default mini-batch size
    end
    if nargin < 10 || isempty(mlpModel)
        mlpModel = []; % Placeholder for MLP model (needs to be trained)
    end
    
    % Ensure Q-table has correct dimensions
    if size(Q,1) < stateSpace || size(Q,2) < numActions
        Q = rand(stateSpace, numActions) * 0.01; % Ensure Q-table is not all zeros
    end
    
    % Ensure state is a valid index
    state = max(1, min(state, stateSpace));
    nextState = max(1, min(nextState, stateSpace));
    
    % Select action using epsilon-greedy policy
    if rand < epsilon
        actionIdx = randi(numActions); % Random action (Exploration)
    else
        qInputs = [state, Q(state, :)]; % Input to MLP (state + current Q-values)
        if ~isempty(mlpModel)
            Q_values = predict(mlpModel, qInputs); % Predict Q-values using MLP
        else
            Q_values = Q(state, :); % Use Q-table if MLP is not available
        end
        [~, actionIdx] = max(Q_values); % Best action (Exploitation)
    end
    action = actionSet{actionIdx};
    
    % Q-learning Update Rule with MLP-NN
    if ~isempty(mlpModel)
        maxNextQ = max(predict(mlpModel, [nextState, Q(nextState, :)]));
    else
        maxNextQ = max(Q(nextState, :));
    end
    targetQ = reward + gamma * maxNextQ;
    Q(state, actionIdx) = Q(state, actionIdx) + alpha * (targetQ - Q(state, actionIdx));
    
    % Mini-batch Training with MLP
    if ~isempty(mlpModel)
        batchStates = randi(size(Q, 1), miniBatch, 1); % Sample random states for training
        batchInputs = [batchStates, Q(batchStates, :)];
        batchTargets = reward + gamma * max(predict(mlpModel, batchInputs), [], 2);
        train(mlpModel, batchInputs, batchTargets); % Train MLP with mini-batch
    end
    
    % Return updated policy
    [~, bestActionIdx] = max(Q(state, :));
    policy = actionSet{bestActionIdx};
end

function reward = computeReward(state)
    % Reward function based on obstacle detection and path planning
    if detectObstacle(state)
        reward = -500; % Large negative reward for collision
    else
        reward = computeThreatValue(state); % Compute threat-based reward
    end
end

function isObstacle = detectObstacle(state)
    % Function to check if an obstacle exists in the current state using MRF
    isObstacle = (state == randi(100, 1)); % Example: Randomly assign obstacles
end

function threatValue = computeThreatValue(state)
    % Function to compute threat value from image segmentation
    threatValue = 1 - sqrt((state - 50)^2) / 50; % Example threat calculation
end

function nextState = getNextState(state, actionSet)
    % Get the next state based on the robot's movement dynamics
    action = actionSet{randi(length(actionSet))};
    switch action
        case 'Move Forward'
            nextState = state + 1;
        case 'Move Reverse'
            nextState = state - 1;
        case 'Turn Left'
            nextState = max(1, state - 5);
        case 'Turn Right'
            nextState = min(100, state + 5);
        otherwise
            nextState = state;
    end
end
