%% Initialization
par.run_type = 'verify';
par = swingup(par);
par.maxtorque = 1.5;
par.a1 = 1;
learner = par.learner;
par = learner.get_parameters(par);

% print(gcf,'foo.png','-dpng','-r450');
    L = 1;
    Font = 11;

disp('Sanity checking robot_learning_control');

%% Parameters
if (par.epsilon <= 0 || par.epsilon >= 1)
    error('Random action rate out of bounds, check get_parameters/par.epsilon');
end
if (par.gamma <= 0 || par.gamma > 1)
    error('Discount rate out of bounds, check get_parameters/par.gamma');
end
if (par.alpha <= 0 || par.alpha > 1)
    error('Learning rate out of bounds, check get_parameters/par.alpha');
end
if (par.pos_states <= 1 || par.pos_states > 1000)
    error('Number of discretized positions out of bounds, check get_parameters/par.pos_states');
end
if (par.vel_states <= 1 || par.vel_states > 1000)
    error('Number of discretized velocities out of bounds, check get_parameters/par.vel_states');
end
if (par.actions <= 1 || par.actions > 100)
    error('Number of actions out of bounds, check get_parameters/par.actions');
end

disp('...Parameters are within bounds');

%% Q value initialization
Q = learner.init_Q(par);
if isempty(Q)
    error('Q value initialization unimplemented, check init_Q');
end

if ndims(Q) ~= 3
    error('Q dimensionality error, check init_Q/Q');
end

if not(all(size(Q)==[par.pos_states, par.vel_states, par.actions]))
    error('Q size error, check init_Q/Q');
end

disp('...Q value dimensionality OK');

%% State discretization
x0 = zeros(1, 2);
s = learner.discretize_state(x0, par);

if isempty(s)
    error('State discretization unimplemented, check discretize_state');
end

disp('...State discretization is implemented');

%% Position discretization
pc = -4*pi:0.05:4*pi;
pd = zeros(size(pc));
for pp = 1:length(pc)
    x0(1) = pc(pp);
    s = learner.discretize_state(x0, par);
    pd(pp) = s(1);
end

if any((pd < 1) | (pd > par.pos_states) | (pd-fix(pd) ~= 0))
    error('Position discretization out of bounds, check discretize_state/s(1)');
end

subaxis(2, 3, 1, 'SV', 0.15, 'SH', 0.1, 'MR', 0.05);
set(gcf, 'name', 'Test results');
plot(pc, pd);
title('Position discretization');
xlabel('Continuous position','FontSize',Font,'FontWeight','bold');
ylabel('Discrete position','FontSize',Font,'FontWeight','bold');
ylim([0 32])

disp('......Position discretization is within bounds');

%% Velocity discretization
vc = -20:0.1:20;
vd = zeros(size(vc));
for vv = 1:length(vc)
    x0(2) = vc(vv);
    s = learner.discretize_state(x0, par);
    vd(vv) = s(2);
end

if any((vd < 1) | (vd > par.vel_states) | (vd-fix(vd) ~= 0))
    error('Velocity discretization out of bounds, check discretize_state/s(2)');
end

subaxis(2, 3, 2);
plot(vc, vd);
title('Velocity discretization');
xlabel('Continuous velocity','FontSize',Font,'FontWeight','bold');
ylabel('Discrete velocity','FontSize',Font,'FontWeight','bold');
ylim([0 32])

disp('......Velocity discretization is within bounds');

%% Action execution
td = learner.take_action(1, par);
if isempty(td)
    error('Action execution unimplemented, check take_action');
end

for aa = 1:par.actions
    td(aa) = learner.take_action(aa, par);
end

if any((td(aa) < -par.maxtorque) | (td(aa) > par.maxtorque))
    error('Action out of bounds, check take_action/TD');
end

subaxis(2, 3, 3)
plot(1:par.actions, td, '.');
title('Action execution');
xlabel('Action','FontSize',Font,'FontWeight','bold');
ylabel('Applied torque','FontSize',Font,'FontWeight','bold');
xlim([0 par.actions])
ylim([floor(-par.maxtorque) ceil(par.maxtorque)])

disp('...Action execution is within bounds');

%% Reward observation
r = learner.observe_reward(1, s, par);
if isempty(r)
    error('Reward observation unimplemented, check observe_reward');
end

r = zeros(par.pos_states,par.vel_states);
for pp = 1:par.pos_states
    for vv = 1:par.vel_states
        for aa = 1:par.actions
            s = [pp vv];
            r(pp,vv,aa) = learner.observe_reward(aa, s, par);
        end
    end
end
r = mean(r, 3)';

subaxis(2, 3, 1, 2, 2, 1);
pcolor([r, r(:,end); r(end,:) r(end,end)]);
title('Average reward');
xlabel('Position','FontSize',Font,'FontWeight','bold');
ylabel('Velocity','FontSize',Font,'FontWeight','bold');
axis tight
colorbar

disp('...Reward observation is implemented');

%% Termination criterion
t = learner.is_terminal(s, par);
if isempty(t)
    error('Termination criterion unimplemented, check is_terminal');
end

t = zeros(par.pos_states,par.vel_states);
for pp = 1:par.pos_states
    for vv = 1:par.vel_states
        s = [pp vv];
        t(pp,vv) = learner.is_terminal(s, par);
    end
end
t = logical(t)+0;
t = t';

subaxis(2, 3, 6);
pcolor([t, t(:,end); t(end,:) t(end,end)]);
title('Termination criterion');
xlabel('Position','FontSize',Font,'FontWeight','bold');
ylabel('Velocity','FontSize',Font,'FontWeight','bold');
axis tight

disp('...Termination criterion is implemented');

%% Policy evaluation
a = learner.execute_policy(Q, s, par);
if isempty(a)
    error('Policy evaluation unimplemented, check execute_policy');
end

for pp = 1:par.pos_states
    for vv = 1:par.vel_states
        s = [pp vv];
        a = learner.execute_policy(Q, s, par);
        if (isempty(a) || a < 1) || (a > par.actions) || (a-fix(a)) ~= 0
            error('Action selection out of bounds, check execute_policy/a');
        end
    end
end

disp('...Action selection is within bounds');

%% Q update rule
r = Q(s(1), s(2), a) * (1 - par.gamma);
Q2 = learner.update_Q(Q, s, a, r, s, a, par);
if (all(all(all(Q == Q2))) ~= 1)
    error('Q update rule unimplemented, check update_Q');
end

disp('...Q update rule is implemented');

disp('Sanity check successfully completed');



