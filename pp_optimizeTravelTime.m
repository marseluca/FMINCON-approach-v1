function [opt_length, opt_alpha, opt_time] = pp_optimizeTravelTime(l_seg, v_max)
    % Inputs:
    % l_seg - Length of the original segment
    % v_max - Maximum velocity
    
    % Initial guesses for the optimization variables
    l_new0 = l_seg / 2;  % Start with half the segment as the new segment
    alpha0 = 0.5;        % Initial guess for the speed scaling factor

    % Lower and upper bounds for the variables
    lb = [0, 0.01];      % l_new >= 0, alpha > 0 (cannot be zero)
    ub = [l_seg, 1];     % l_new <= l_seg, alpha <= 1
    
    % Define the objective function
    objective = @(x) (l_seg - x(1)) / v_max + (x(1) / (x(2) * v_max));

    % Options for fmincon
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

    % Run the optimization
    [opt_vars, opt_time] = fmincon(objective, [l_new0, alpha0], [], [], [], [], lb, ub, [], options);

    % Extract the optimized values
    opt_length = opt_vars(1);
    opt_alpha = opt_vars(2);
end
