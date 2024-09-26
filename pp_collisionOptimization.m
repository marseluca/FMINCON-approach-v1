% Define constants and inputs

global maxVelocity collision_position delta_t;
global x_opt;

% Maximum velocity of the robot
safetyMargin = 20;
delta_t = safetyMargin / maxVelocity; % Minimum time difference to avoid collision

% Define the objective function: minimize total travel time
function total_time = objective(x)
    global L maxVelocity collision_position
    
    % Extract optimization variables
    L_s = x(1);  % Length of the slow-down segment
    alpha = x(2); % Scaling factor for slowing down
    
    % Ensure alpha is not zero or too small to prevent division by zero
    if alpha < 0.01
        alpha = 0.01; % Set a small minimum value for alpha
    end
    
    % Ensure L_s is within valid bounds
    if L_s < 0
        L_s = 0; % No negative segment length
    elseif L_s > collision_position
        L_s = collision_position; % Max length is up to the collision position
    end
    
    % Time spent in the slow-down segment
    time_slow = L_s / (alpha * maxVelocity);
    
    % Time spent in the remaining segment
    % time_fast = (collision_position - L_s) ./ v_max;
    time_fast = (L - L_s) / maxVelocity;
    
    % Total time to reach the end of  the segment
    total_time = time_slow + time_fast;
    
    % Ensure total_time is scalar
    if numel(total_time) > 1
        error('Objective function must return a scalar value, but received an array.');
    end
end




% Define the collision avoidance constraint
function [c, ceq] = collision_constraint(x)
    global maxVelocity L collision_position delta_t absoluteCollidingTime;

    L_s = x(1);
    alpha = x(2);
    
    % Time for the robot that slows down
    time_robot1 = (L_s / (alpha * maxVelocity)) + ((collision_position - L_s) / maxVelocity);
    
    % Time for the other robot (assumed to move at full speed)
    time_robot2 = absoluteCollidingTime;
    
    % Collision constraint
    % T1 >= DELTA + T2 così da rallentare T1 ed evitare la collisione
    % Quindi T1 - DELTA - T2 >= 0 --> c = T2 + DELTA - T1 <= 0 
    c = delta_t - time_robot1 + time_robot2; % Time buffer to avoid collision
    ceq = [];
end

% Initial guess for the optimization variables [L_s, alpha]
x0 = [collision_position / 2, 0.5];

% Bounds: 0 < alpha <= 1, and L_s should be between 0 and the collision position
lb = [0, 0.1]; % Lower bounds for [L_s, alpha]
ub = [collision_position, 1]; % Upper bounds for [L_s, alpha]

% Set optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% Call the fmincon solver
[x_opt, fval] = fmincon(@objective, x0, [], [], [], [], lb, ub, @collision_constraint, options);

% Call the collision constraint function with the optimal solution
[c_val, ceq_val] = collision_constraint(x_opt);

% Output results
fprintf('Optimal length of slow-down segment: %f\n', x_opt(1));
fprintf('Optimal velocity scaling factor: %f\n', x_opt(2));
fprintf('Minimized travel time: %f\n', fval);

% Output the value of c
fprintf('Value of c (constraint): %f\n', c_val);
