function rp = define_robot_parameters()
    rp.l1 = 2;    % (m)Length of element 1
    rp.lc1 = 1;   %(m) Distance of CM of element 1 from source end
    rp.m1 = 10;   % (kg) Mass of element 1
    rp.j1 = 3;    % (kg-m^2) Moment of Inertia of element 1 about its CM

    rp.l2 = 1;    % (m)Length of element 2
    rp.lc2 = 0.5; % (m)Distance of CM of element 2 from source end
    rp.m2 = 6;    % (kg) Mass of element 2
    rp.j2 = 2;    % (kg-m^2)Moment of Inertia of element 2 about its CM

    rp.g = 9.81;  % (m/sec^2) Acceleration due to gravity
end