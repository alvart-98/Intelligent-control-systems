%%%% Dynamics Controller (Feedback Part)
function tau_des_fb = fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd)
    % simple PD controller on joints
    tau_des_fb = Kp.*(th_des-th_curr) + Kd.*(th_d_des-th_d_curr);
    % tau_des_fb = [0; 0];
end