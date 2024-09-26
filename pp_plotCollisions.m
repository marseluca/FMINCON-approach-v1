function result = pp_plotCollisions(collisions,trajectories)
    
    global pathColors;
    
    figure(1)
    if ~isempty(collisions) 
        for i=1:size(collisions,2)
            for j=1:size(collisions{i},1)
                k = collisions{i}(j,2);
                plot(trajectories{i}.x_tot(k),trajectories{i}.y_tot(k),"o","Color",pathColors(i,:),"MarkerSize",8);
            end
        end
    end

end

