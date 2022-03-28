function curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ctr_ff, ctr_fb)
    curr.x     = zeros(2,length(t));
    curr.x_d   = zeros(2,length(t));
    curr.x_dd  = zeros(2,length(t));
    curr.x_eb  = zeros(2,length(t));
    curr.th    = zeros(2,length(t));
    curr.th_d  = zeros(2,length(t));
    curr.th_dd = zeros(2,length(t));
    curr.tau_ff = zeros(2,length(t));
    curr.tau_ff = zeros(2,length(t));
    
    curr.th(:,1)   = th_0;
    curr.th_d(:,1) = th_d_0;
    [curr.x(:,1), curr.x_d(:,1), curr.x_dd(:,1), curr.x_eb(:,1)] = FK(th_0, th_d_0, [0; 0], rp);

    for iter = 2:length(t)
        th_curr = curr.th(:,iter-1); th_d_curr = curr.th_d(:,iter-1);
        th_des = des.th(:,iter); th_d_des = des.th_d(:,iter); th_dd_des = des.th_dd(:,iter);
        
        % calculate control
        tau_ff = ctr_ff(th_curr, th_d_curr, th_des, th_d_des, th_dd_des);
        tau_fb = ctr_fb(th_curr, th_d_curr, th_des, th_d_des);
        
        %% Dynamics matrices: Current
        [M_curr, V_curr, G_curr] = RBD_matrices(th_curr, rp);
        
        %% Deriving angular acc
        th_dd_new = M_curr\(tau_ff + tau_fb - V_curr*(th_d_curr.^2) - G_curr);
        
        %% Simulation Update, symplectic Euler integration
        th_d_new = th_d_curr + dt*th_dd_new;
        th_new = th_curr + dt*th_d_new;

        %% write to structs
        curr.th(:,iter) = th_new; curr.th_d(:,iter) = th_d_new; curr.th_dd(:,iter) = th_dd_new;
        curr.tau_ff(:,iter) = tau_ff; curr.tau_fb(:,iter) = tau_fb;
        
        %% Convert to Cartesian values
        [curr.x(:,iter), curr.x_d(:,iter), curr.x_dd(:,iter), curr.x_eb(:,iter)] = FK(th_new, th_d_new, th_dd_new, rp);
    end
end

