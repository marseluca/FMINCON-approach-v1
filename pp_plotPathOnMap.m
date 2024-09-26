function plotOnMap = pp_plotPathOnMap(paths,trajectories,style)
    
    global nRobots;
    global pathColors;

    figure(1)
    for j=1:nRobots
        plot(paths{j}(1,1),paths{j}(1,2),'square','MarkerSize',7,'MarkerFaceColor',pathColors(j,:))
        plot(paths{j}(end,1),paths{j}(end,2),'square','MarkerSize',7,'MarkerFaceColor','r')
        text(paths{j}(1,1),paths{j}(1,2),num2str(j),'Color','black');

        x = paths{j}(:,1);
        y = paths{j}(:,2);
        plot(x, y,'o','MarkerFaceColor',pathColors(j,:),'MarkerSize',3);
        plot(trajectories{j}.x_tot,trajectories{j}.y_tot,style,'Color',pathColors(j,:),'LineWidth',1.2);
             
    end


    set(gca,'XAxisLocation','top', 'box','off', 'XTick', [])
    set(gca,'YAxisLocation','left', 'box','off', 'YTick', [])
    xlabel("");
    ylabel("");
    axis tight;

end

