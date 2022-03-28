%%%% Dynamics Controller
function tau_ff = ff_yours_training(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, your_parameters)
%     Function to implement feed-forward controller to guide system during the 
%     collection of training data. The output is tau_ff, 
%     which is the actuation torque on the system.
%
%     the robot draws the ellipse approx. twice, consider wrapping the
%     angles to get better training data
%     
%     if you want to use Cartesian positions instead of joint positions in
%     your function approximator:
%     [x_des, x_d_des, x_dd_des, ~] = FK(th_des, th_d_des, th_dd_des, rp);
%     and this is the only purpose for which you are allowed to use the
%     robot parameters rp.

%%  TODO: Guide training data generation using Tau 
    tau_ff = [0; 0];
end