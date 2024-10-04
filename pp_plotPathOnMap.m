function plotOnMap = pp_plotPathOnMap(paths,trajectories,style)
    
    global nRobots;
    global pathColors;
    global x_opt;

    figure(1)
    for j=1:nRobots
        plot(paths{j}(1,1),paths{j}(1,2),'square','MarkerSize',7,'MarkerFaceColor',pathColors(j,:))
        plot(paths{j}(end,1),paths{j}(end,2),'square','MarkerSize',7,'MarkerFaceColor','r')
        text(paths{j}(1,1),paths{j}(1,2),num2str(j),'Color','black');

        x = paths{j}(:,1);
        y = paths{j}(:,2);
        plot(x, y,'o','MarkerFaceColor',pathColors(j,:),'MarkerSize',3);
        
        if ~isempty(x_opt) && x_opt(3*(j-1) + 1)==1
            for k=1:length(trajectories{j}.x_tot)
                if norm([trajectories{j}.x_tot(k),trajectories{j}.y_tot(k)]-[x(2),y(2)])<0.5
                    break;
                end
            end
            
            plot(trajectories{j}.x_tot(1:k),trajectories{j}.y_tot(1:k),'--','Color',pathColors(j,:),'LineWidth',1);
            plot(trajectories{j}.x_tot(k+1:end),trajectories{j}.y_tot(k+1:end),style,'Color',pathColors(j,:),'LineWidth',1.2);
        else
            plot(trajectories{j}.x_tot,trajectories{j}.y_tot,style,'Color',pathColors(j,:),'LineWidth',1.2);
        end
    end


    set(gca,'XAxisLocation','top', 'box','off', 'XTick', [])
    set(gca,'YAxisLocation','left', 'box','off', 'YTick', [])
    xlabel("");
    ylabel("");
    axis tight;

end

