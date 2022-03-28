function par = robot_set_parameters
%% initialize body variables

par.gamma = 0;
par.g = 9.81;
par.m1 = 2;
par.m2 = 2;
par.m3 = 5; 
par.m4 = 2;
par.a1 = 1;
par.a2 = 1;
par.a3 = 1;
par.a4 = 0.7;
par.c1 = 0.2;
par.c2 = 0.2;
par.c3 = 0.2;
par.c4 = 0.2;
par.I1 = (1/15)*par.m1*par.a1^2;
par.I2 = (1/15)*par.m2*par.a2^2;
par.I3 = (1/15)*par.m3*par.a3^2;
par.I4 = (1/15)*par.m4*par.a4^2;
