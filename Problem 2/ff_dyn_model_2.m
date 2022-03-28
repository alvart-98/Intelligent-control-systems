%%%% Dynamics Controller
function tau_ff = ff_dyn_model_2(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, rp)
    %% Dynamics matrices
    [M_des, V_des, G_des] = RBD_matrices(th_des,rp);
    tau_ff = M_des*(th_dd_des) + V_des*(th_d_des.^2) + G_des;
end