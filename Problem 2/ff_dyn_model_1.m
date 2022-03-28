%%%% Dynamics Controller
function tau_ff = ff_dyn_model_1(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, rp)
    %% Dynamics matrices
    [M_curr, V_curr, G_curr] = RBD_matrices(th_curr,rp);
    tau_ff = M_curr*(th_dd_des) + V_curr*(th_d_curr.^2) + G_curr;
end