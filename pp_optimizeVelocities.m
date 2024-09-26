function closest_vector = pp_optimizeVelocities(vectors, alpha, beta, targetVelocity)
    
    % This function finds the velocities that are:
    % - Most similar between each other
    % - Closer to 20

    % Number of vectors
    num_vectors = size(vectors, 1);
    
    % Initialize the minimum score and the index of the closest vector
    min_score = inf;
    closest_idx = 1;
    
    % Loop over each vector
    for i = 1:num_vectors
        % Current vector
        current_vector = vectors(i, :);
        
        % Calculate the sum of absolute differences from target velocity
        diff_sum = sum(abs(current_vector - targetVelocity));
        
        % Calculate the variance of the elements in the vector
        variance = var(current_vector);
        
        % Objective function: Weighted sum of deviations and variance
        score = alpha * diff_sum + beta * variance;
        
        % Update the closest vector if the current score is lower
        if score < min_score
            min_score = score;
            closest_idx = i;
        end
    end
    
    % Return the vector with the lowest score
    closest_vector = vectors(closest_idx, :);
end

