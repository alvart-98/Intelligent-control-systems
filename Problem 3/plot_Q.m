function plot_Q(Q, par, ra, tta, ii)
    % Visualization
    [val, pos] = max(Q, [], 3);
    val = val';
    pos = movavg(pos', 3);
    
    % print(gcf,'foo.png','-dpng','-r450');
    L = 1;
    Font = 11;
    
    % Value function
    subaxis(2, 2, 1, 'MR', 0.05, 'SV', 0.15, 'SH', 0.1);
    pcolor([val, val(:,end); val(end,:) val(end,end)]);
    axis tight
    title('State-value function (V = max_a(Q(s, a)))');
    xlabel('Position','FontSize',Font,'FontWeight','bold');
    ylabel('Velocity','FontSize',Font,'FontWeight','bold');

    % Policy
    subaxis(2, 2, 2);
    pcolor([pos, pos(:,end); pos(end,:) pos(end,end)]);
    axis tight
    title('Policy (argmax_a(Q(s, a)))');
    xlabel('Position','FontSize',Font,'FontWeight','bold');
    ylabel('Velocity','FontSize',Font,'FontWeight','bold');

    % Reward
    subaxis(2, 2, 3);
    plot(1:ii, movavg(ra(1:ii), 9));
    set(gca, 'xlim', [0 par.trials]);
    title('Progress');
    xlabel('Trial','FontSize',Font,'FontWeight','bold');
    ylabel('Average cumulative reward','FontSize',Font,'FontWeight','bold');

    % Trial duration
    subaxis(2, 2, 4);
    plot(movavg(tta(1:ii), 9));
    set(gca, 'xlim', [0 par.trials]);
    title('Progress');
    xlabel('Trial','FontSize',Font,'FontWeight','bold');
    ylabel('Average trial duration','FontSize',Font,'FontWeight','bold');

    set(gcf, 'Name', ['Iteration ' num2str(ii)]);
end
