function all_combinations = pp_velocitiesCombinations(nRobots,maxVel,velStep)
        
    % Create n vectors, each with m elements
    vectors = cell(1, nRobots);
    for i = 1:nRobots
        vectors{i} = maxVel:-velStep:2; 
    end
    
    % Generate all combinations using ndgrid and reshape
    combinations = cell(1, nRobots);
    [combinations{:}] = ndgrid(vectors{:});
    combinations = cellfun(@(x) x(:), combinations, 'UniformOutput', false);
    all_combinations = [combinations{:}];
    
    % Cycle through all combinations
    % for i = 1:size(all_combinations, 1)
    %     % Display or process each combination
    %     disp(all_combinations(i, :));
    % end

end

