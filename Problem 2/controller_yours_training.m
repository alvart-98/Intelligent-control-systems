clc
clearvars
close all

% tau - torques applied to joints
% th - positions of the joints (angles)
% th_d - velocities of the joints (angular velocity)
% th_dd - acceleration of the joints
% _des - desired values (reference)
% _curr - current values (measured)
% ff_ - feedforward
% fb_ - feedback

rp = define_robot_parameters();
sim_time = 10; % simualtion time in seconds
dt = 0.03; % time difference in seconds
t = 0:dt:sim_time;

dataset = [];
Ntests = 10;
w_min = 70;
w_max = 80;

for i = 0:Ntests

    %% DESIRED TRAJECTORY DATA
    d2r  = pi/180;             % degrees to radians
    tp.w = ((w_max-w_min)*i/Ntests+w_min)*d2r;            % rotational velocity rad/s (72 for basic case)
    tp.rx = 1.75; tp.ry = 1.25; % ellipse radii
    tp.ell_an = 45*d2r;       % angle of inclination of ellipse
    tp.x0 = 0.4;  tp.y0 = 0.4;  % center of ellipse  

    % Calculate desired trajectory in task space and in joint space
    des = calculate_trajectory(t, tp, rp);

    th_0 = des.th(:,1) - [0.1; 0.2];
    th_d_0 = des.th_d(:,1);

    % Your Code
    your_parameters = [];

    %% SIMULATE ROBOT
    Kp = [500; 500];
    Kd = [50; 50];
    % curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
    %    @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_yours_training(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, your_parameters), ...
    %    @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));
    curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
       @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_dyn_model_2(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, rp), ...
       @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));

    % fit_some_model(des, curr)

    % robot_animation(t, curr, des);
    % analyze_performance(t, curr, des);
    datalocal = [wrapToPi(curr.th)',curr.th_d',curr.th_dd',curr.tau_ff'];
    dataset = [dataset;datalocal(2:end,:)];

    % function fit_some_model(des, curr)
    %
    %    model = []; % select some method to fit a model
    %    save model_yours model
    %
    % end
end

save 'dataset.dat' dataset -ascii   % save to myfile.dat 
load dataset.dat   % load the file

%% Training ANFIS

opt = anfisOptions('InitialFIS',2,'EpochNumber',15);
opt.DisplayErrorValues = 1;
opt.DisplayStepSize = 1;
fis1 = anfis(dataset(:,1:end-1),opt);
fis2 = anfis([dataset(:,1:end-2),dataset(:,end)],opt);
model.F1 = fis1;
model.F2 = fis2;
save 'model_yours.mat' model
