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

%% DESIRED TRAJECTORY DATA
d2r  = pi/180;             % degrees to radians
tp.w = 72*d2r;            % rotational velocity rad/s
tp.rx = 1.75; tp.ry = 1.25; % ellipse radii
tp.ell_an = 45*d2r;       % angle of inclination of ellipse
tp.x0 = 0.4;  tp.y0 = 0.4;  % center of ellipse  

% Calculate desired trajectory in task space and in joint space
des = calculate_trajectory(t, tp, rp);

th_0 = des.th(:,1) - [0.1; 0.2];
th_d_0 = des.th_d(:,1);

%% SIMULATE ROBOT
% Kp = [6500; 4500];
% Kd = [150; 100];
% Kp = [6000; 4000]; 
% Kd = [150; 100];
% Kp = [6500; 4500]; %The best
% Kd = [90; 70];
% Original:
Kp = [2000; 2000];
Kd = [100; 100];
curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
    @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_0(th_curr, th_d_curr, th_des, th_d_des, th_dd_des), ...
    @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));

robot_animation(t, curr, des);
analyze_performance(t, curr, des);