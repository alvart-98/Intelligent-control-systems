%%% Calculates the Jacobian of the Robot
function J = Jacobian(th,rp)
    J = [-rp.l1*sin(th(1)) -rp.l2*sin(th(2));
          rp.l1*cos(th(1))  rp.l2*cos(th(2))];
end