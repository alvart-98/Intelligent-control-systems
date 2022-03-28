%%% Calculates the derivative of the Jacobian of the Robot
function J_d = Jacobian_dot(th,th_d,rp)
    J_d = [-rp.l1*cos(th(1))*th_d(1), -rp.l2*cos(th(2))*th_d(2);
           -rp.l1*sin(th(1))*th_d(1), -rp.l2*sin(th(2))*th_d(2)];
end