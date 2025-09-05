classdef Lidarsensor < matlab.System
    % Lidar Sensor System Object for DQN Multi-Robot Navigation in a Warehouse

    properties
        NumRays = 10;       % Number of Lidar beams
        MaxRange = 5.0;     % Max sensing range (meters)
        FieldOfView = 180;  % Field of view (degrees)
        NoiseLevel = 0.05;  % Gaussian noise in distance readings
    end
    
    methods (Access = protected)
        function distances = stepImpl(obj, robotPose, obstacles)
            % Simulates Lidar scanning from the robot's position
            % robotPose: [x, y, theta] (robot position and heading)
            % obstacles: Nx3 matrix [x, y, radius] for obstacles
            
            numBeams = obj.NumRays;
            angles = linspace(-obj.FieldOfView/2, obj.FieldOfView/2, numBeams);
            distances = obj.MaxRange * ones(1, numBeams); % Default max range
            
            for i = 1:numBeams
                rayAngle = deg2rad(angles(i)) + robotPose(3);
                rayEnd = [robotPose(1) + obj.MaxRange * cos(rayAngle), ...
                          robotPose(2) + obj.MaxRange * sin(rayAngle)];
                
                for j = 1:size(obstacles, 1)
                    obsPos = obstacles(j, 1:2);
                    obsRadius = obstacles(j, 3);
                    
                    d = obj.checkIntersection(robotPose(1:2), rayEnd, obsPos, obsRadius);
                    if ~isempty(d) && d < distances(i)
                        distances(i) = d;
                    end
                end
                
                % Add Gaussian noise to Lidar readings
                distances(i) = max(0, min(obj.MaxRange, distances(i) + obj.NoiseLevel * randn));
            end
        end
    end
    
    methods (Access = private)
        function d = checkIntersection(~, p1, p2, obsPos, obsRadius)
            % Check if Lidar ray (p1->p2) intersects with circular obstacle
            p1 = p1(:); p2 = p2(:); obsPos = obsPos(:);
            d = [];
            
            dirVec = p2 - p1;
            f = p1 - obsPos;
            
            a = dot(dirVec, dirVec);
            b = 2 * dot(f, dirVec);
            c = dot(f, f) - obsRadius^2;
            
            discriminant = b^2 - 4*a*c;
            if discriminant >= 0
                t1 = (-b - sqrt(discriminant)) / (2*a);
                t2 = (-b + sqrt(discriminant)) / (2*a);
                t = min([t1, t2, 1], [], 'omitnan');
                if t > 0
                    d = norm(t * dirVec);
                end
            end
        end
    end
end
