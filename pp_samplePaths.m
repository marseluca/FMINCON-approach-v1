function sampledPaths = pp_samplePaths(paths)
    
    global nRobots
    nSamples = 10;
    sampledPaths = {};

    for j=1:nRobots
         x = paths{j}(:,1)';
         y = paths{j}(:,2)';
         x_values = [];
         y_values = [];

         for i=2:size(x,2)

            x_values = [x_values; linspace(x(i-1), x(i), nSamples)'];
            y_values = [y_values; linspace(y(i-1), y(i), nSamples)'];

         end

        sampledPaths{j}(:,1) = x_values;
        sampledPaths{j}(:,2) = y_values;
    end
    
end

