function angle = computeAngle(P1, P2, P3)

    % This function computes the angle between the two lines
    % formed by the points P1-P2 and P2-P3
    
    % Define the vectors
    v1 = P2 - P1;
    v2 = P3 - P2;
    
    % Compute the dot product of the two vectors
    dot_product = dot(v1, v2);
    
    % Compute the magnitudes (norms) of the vectors
    norm_v1 = norm(v1);
    norm_v2 = norm(v2);
    
    % Compute the cosine of the angle using the dot product formula
    cos_theta = dot_product / (norm_v1 * norm_v2);
    
    % Clamp cos_theta to the range [-1, 1] to avoid numerical issues with acos
    cos_theta = max(min(cos_theta, 1), -1);
    
    % Compute the angle in radians
    angle_radians = acos(cos_theta);
    
    % Convert the angle to degrees
    angle_degrees = rad2deg(angle_radians);
    
    % Return the angle in degrees
    angle = angle_degrees;
end

