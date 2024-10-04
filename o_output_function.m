function [state, options] = o_output_function(options, state, flag)
    % Only plot at the end of each generation
    if strcmp(flag, 'iter')
        % Here you can access the current best fitness value
        current_best = state.Score(1);
        
        % Generate a plot (for example, fitness over generations)
        figure(1);
        plot(state.Generation, current_best, 'bo'); % Plot current best fitness
        hold on;

        % Optionally, plot all fitness values if needed
        % plot(state.Generation, state.Score, 'ro');

        title('Best Fitness Over Generations');
        xlabel('Generation');
        ylabel('Best Fitness Value');
        drawnow; % Update the figure window
    end
    
    % Output the state unchanged
    state = options;
end