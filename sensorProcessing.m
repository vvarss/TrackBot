%%function obsData = sensorProcessing(lidarData, cameraData)
%%    if nargin < 2
%%        lidarData = 1;  % Default value
%%        cameraData = 1; % Default value
%%    end
%%    threshold = 0.5;
%%    obsData = (lidarData < threshold) | (cameraData < threshold);
%% end
%{
function action = sensorProcessing(lidarData)
    % Handle missing inputs
    if nargin < 1
        lidarData = [1.5, 0.8, 2.0, 1.2]; % Default LiDAR readings (distances)
        disp("⚠️ No inputs provided. Using default LiDAR data.");
    end

    % Define action selection logic
    safeDistance = 1.0; % Safety threshold
    if min(lidarData) < safeDistance
        action = 4; % Move right if obstacle detected
    else
        action = 1; % Move up if clear
    end

    % Display for debugging
    disp("📌 LiDAR Data: " + mat2str(lidarData) + " | Chosen Action: " + num2str(action));
end
%}
function obstacleDetected = sensorProcessing(robotPos)
    % Simulated sensor processing for obstacle detection
    % Inputs:
    %   robotPos - Current [x, y] position of the robot
    % Output:
    %   obstacleDetected - Boolean indicating if an obstacle is present
    
    % Default value if no input is provided
    if nargin < 1 || isempty(robotPos)
        robotPos = [50, 50]; % Default position
    end
    
    % Define obstacle locations (can be replaced with LiDAR input)
    obstaclePositions = [
        20, 30;
        40, 50;
        60, 70;
        80, 90;
        25, 75;
        55, 35;
        75, 25;
        95, 85
    ];
    
    % Check if the robot is near an obstacle
    obstacleDetected = any(vecnorm(obstaclePositions - robotPos, 2, 2) < 5);
    
    % Automatically execute if script runs independently
    if nargout == 0
        if obstacleDetected
            disp(['Obstacle detected at position [', num2str(robotPos(1)), ', ', num2str(robotPos(2)), ']']);
        else
            disp(['No obstacle detected at position [', num2str(robotPos(1)), ', ', num2str(robotPos(2)), ']']);
        end
    end
end
