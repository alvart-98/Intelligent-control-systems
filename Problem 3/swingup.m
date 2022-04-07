function [par, ta, xa] = swingup(par)

    par.simtime = 10;     % Trial length
    par.simstep = 0.05;   % Simulation time step
    par.maxtorque = 1.5;  % Maximum applicable torque
    
    if strcmp(par.run_type, 'learn')
        %%
        % Obtain SARSA parameters
        par = get_parameters(par);
        
		% TODO: Initialize the outer loop
        Q = init_Q(par);
        
        % Initialize bookkeeping (for plotting only)
        ra = zeros(par.trials, 1);
        tta = zeros(par.trials, 1);
        te = 0;

        % Outer loop: trials
        for ii = 1:par.trials

            % TODO: Initialize the inner loop
            x = swingup_initial_state();
            s = discretize_state(x, par);
            a = execute_policy(Q, s, par);
            
            % Inner loop: simulation steps
            for tt = 1:ceil(par.simtime/par.simstep)
                
                % TODO: obtain torque
                u = take_action(a, par);
                
                % Apply torque and obtain new state
                % x  : state (input at time t and output at time t+par.simstep)
                % u  : torque
                % te : new time
                [te, x] = pendulum_dynamics([te te+par.simstep],x,u,par);

                % TODO: learn
                % use s for discretized state
                sP = discretize_state(x, par);
                aP = execute_policy(Q, sP, par);
                reward = observe_reward(a, sP, par);
                Q = update_Q(Q, s, a, reward, sP, aP, par);
                s = sP;
                a = aP;
                
                % Keep track of cumulative reward
                ra(ii) = ra(ii)+reward;

                % TODO: check termination condition
                t = is_terminal(sP, par);
                if t == 1
                    break;
                end
            end

            tta(ii) = tta(ii) + tt*par.simstep;

            % Update plot every ten trials
            if rem(ii, 10) == 0
                plot_Q(Q, par, ra, tta, ii);
                drawnow;
            end
        end
        
        % save learned Q value function
        par.Q = Q;
 
    elseif strcmp(par.run_type, 'test')
        %%
        % Obtain SARSA parameters
        par = get_parameters(par);
        
        % Read value function
        Q = par.Q;
        
        x = swingup_initial_state();
        
        ta = zeros(length(0:par.simstep:par.simtime), 1);
        xa = zeros(numel(ta), numel(x));
        te = 0;
        
        % Initialize a new trial
        s = discretize_state(x, par);
        a = execute_policy(Q, s, par);

        % Inner loop: simulation steps
        for tt = 1:ceil(par.simtime/par.simstep)
            % Take the chosen action
            TD = max(min(take_action(a, par), par.maxtorque), -par.maxtorque);

            % Simulate a time step
            [te,x] = pendulum_dynamics([te te+par.simstep],x,TD,par);

            % Save trace
            ta(tt) = te;
            xa(tt, :) = x;

            s = discretize_state(x, par);
            a = execute_policy(Q, s, par);

            % Stop trial if state is terminal
            if is_terminal(s, par)
                break
            end
        end

        ta = ta(1:tt);
        xa = xa(1:tt, :);
        
    elseif strcmp(par.run_type, 'verify')
        %%
        % Get pointers to functions
        learner.get_parameters = @get_parameters;
        learner.init_Q = @init_Q;
        learner.discretize_state = @discretize_state;
        learner.execute_policy = @execute_policy;
        learner.observe_reward = @observe_reward;
        learner.is_terminal = @is_terminal;
        learner.update_Q = @update_Q;
        learner.take_action = @take_action;
        par.learner = learner;
    end
    
end

% ******************************************************************
% *** Edit below this line                                       ***
% ******************************************************************
function par = get_parameters(par)
    % TODO: set the values
    par.epsilon = 0.1;        % Random action rate
    par.gamma = 0.99;       % Discount rate
    par.alpha = 0.25;          % Learning rate
    par.pos_states = 31;     % Position discretization
    par.vel_states = 31;     % Velocity discretization
    par.actions = 5;        % Action discretization
    par.trials = 2000;         % Learning trials
end

function Q = init_Q(par)
    % TODO: Initialize the Q table.
%     Q = zeros(par.pos_states,par.vel_states,par.actions);
    Q = 1*ones(par.pos_states,par.vel_states,par.actions);
end

function s = discretize_state(x, par)
    % TODO: Discretize state. Note: s(1) should be
    % TODO: position, s(2) velocity.
    step1 = 2*pi/(par.pos_states);
    s(1) = floor(wrapTo2Pi(x(1))/step1)+1;
    step2 = 10*pi/(par.pos_states);
    if x(2) <= -5*pi
        s(2) = 1;
    elseif x(2) >= 5*pi
        s(2) = par.vel_states;
    else
        s(2) = floor((x(2)+5*pi)/step2)+1;
    end
end

function u = take_action(a, par)
    % TODO: Calculate the proper torque for action a. This cannot
    % TODO: exceed par.maxtorque.
    step3 = 2*par.maxtorque/(par.actions-1);
    u = (a-1)*step3-par.maxtorque;
end

function r = observe_reward(a, sP, par)
    % TODO: Calculate the reward for taking action a,
    % TODO: resulting in state sP.
    if (sP(1) == ceil(par.pos_states/2)) & (sP(2) == ceil(par.vel_states/2)) %(a == ceil(par.actions/2)) & 
        r = 10;   
    else
        r = 0;
    end
end

% Second option for reward function:
% function r = observe_reward(a, sP, par)
%     % TODO: Calculate the reward for taking action a,
%     % TODO: resulting in state sP.
%     if (a == ceil(par.actions/2)) & (sP(1) == ceil(par.pos_states/2)) & (sP(2) == ceil(par.vel_states/2)) 
%         r = 50;
%     elseif (a >= ceil(par.actions*3/4)) & (sP(1) <= ceil(par.pos_states*1/4)) & (sP(2) >= ceil(par.vel_states/2))
%         r = 2;
%     elseif (a <= ceil(par.actions*1/4)) & (sP(1) >= ceil(par.pos_states*3/4)) & (sP(2) <= ceil(par.vel_states/2))
%         r = 2;
%     elseif (a <= ceil(par.actions*1/2) & a >= ceil(par.actions*1/4)) & (sP(1) >= ceil(par.pos_states*1/4) & sP(1) <= ceil(par.pos_states*1/2)) & (sP(2) >= ceil(par.vel_states/2))
%         r = 1;
%     elseif (a <= ceil(par.actions*3/4) & a >= ceil(par.actions*1/2)) & (sP(1) >= ceil(par.pos_states*1/2) & sP(1) <= ceil(par.pos_states*3/4)) & (sP(2) <= ceil(par.vel_states/2))
%         r = 1;
%     else
%         r = 0;
%     end
% end

function t = is_terminal(sP, par)
    % TODO: Return 1 if state sP is terminal, 0 otherwise.
    if (sP(1) == ceil(par.pos_states/2)) & (sP(2) == ceil(par.vel_states/2)) 
        t = 1;
    else
        t = 0;
    end
end

% Epsilon-greedy policy
function a = execute_policy(Q, s, par)
    % TODO: Select an action for state s using the
    % TODO: epsilon-greedy algorithm.
    pos_a = find(Q(s(1),s(2),:) == max(Q(s(1),s(2),:)));
    if length(pos_a) > 1
        pos_a_a = round((length(pos_a)-1)*rand(1)+1);
        pos_a = pos_a(pos_a_a);
    end
    if rand(1) <= par.epsilon
        a = round((par.actions-1)*rand(1)+1);
    else
        a = pos_a;
    end
end

% Greedy policy
% function a = execute_policy(Q, s, par)
%     % TODO: Select an action for state s using the
%     % TODO: greedy algorithm.
%     pos_a = find(Q(s(1),s(2),:) == max(Q(s(1),s(2),:)));
%     if length(pos_a) > 1
%         pos_a_a = round((length(pos_a)-1)*rand(1)+1);
%         pos_a = pos_a(pos_a_a);
%     end
%     a = pos_a;
% end

function Q = update_Q(Q, s, a, r, sP, aP, par)
    % TODO: Implement the SARSA update rule.
    Q(s(1),s(2),a) = Q(s(1),s(2),a)+par.alpha*(r+par.gamma*Q(sP(1),sP(2),aP)-Q(s(1),s(2),a));
end
