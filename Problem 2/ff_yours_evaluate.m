%%%% Dynamics Controller
function tau_ff = ff_yours_evaluate(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, model)
%     Function to implement feed-forward controller obtained from a 
%     data-driven method. The output is tau_ff, which is the 
%     actuation torque on the system.

%     the robot draws the ellipse approx. twice, consider wrapping the
%     angles to get better training data
%     
%     if you want to use Cartesian positions instead of joint positions in
%     your function approximator:
%     [x_des, x_d_des, x_dd_des, ~] = FK(th_des, th_d_des, th_dd_des, rp);
%     and this is the only purpose for which you are allowed to use the
%     robot parameters rp.
%     

%%  TODO: load your model
%     load model_yours model
    states = [wrapToPi(th_des)',th_d_des',th_dd_des'];
    tau_ff = [evalfis(model.F1,states); evalfis(model.F2,states)];
%     states = [wrapToPi(th_curr)',th_d_curr',wrapToPi(th_des)',th_d_des',th_dd_des'];
%     tau_ff = [evalfis(model.F1,states([1,3,5,7,9])); evalfis(model.F2,states([2,4,6,8,10]))];
end