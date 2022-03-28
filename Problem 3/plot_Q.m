function plot_Q(Q, par, ra, tta, ii)
    % Visualization
    [val, pos] = max(Q, [], 3);
    val = val';
    pos = movavg(pos', 3);

    % Value function
    subaxis(2, 2, 1, 'MR', 0.05, 'SV', 0.15, 'SH', 0.1);
    pcolor([val, val(:,end); val(end,:) val(end,end)]);
    axis tight
    title('State-value function (V = max_a(Q(s, a)))');
    xlabel('Position');
    ylabel('Velocity');

    % Policy
    subaxis(2, 2, 2);
    pcolor([pos, pos(:,end); pos(end,:) pos(end,end)]);
    axis tight
    title('Policy (argmax_a(Q(s, a)))');
    xlabel('Position');
    ylabel('Velocity');

    % Reward
    subaxis(2, 2, 3);
    plot(1:ii, movavg(ra(1:ii), 9));
    set(gca, 'xlim', [0 par.trials]);
    title('Progress');
    xlabel('Trial');
    ylabel('Average cumulative reward');

    % Trial duration
    subaxis(2, 2, 4);
    plot(movavg(tta(1:ii), 9));
    set(gca, 'xlim', [0 par.trials]);
    title('Progress');
    xlabel('Trial');
    ylabel('Average trial duration');

    set(gcf, 'Name', ['Iteration ' num2str(ii)]);
end