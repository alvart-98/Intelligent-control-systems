%%% calculate forward kinematics
function [x, x_d, x_dd, x_eb] = FK(th, th_d, th_dd, rp)
    % ellbow position
    x_eb = [rp.l1*cos(th(1));
            rp.l1*sin(th(1))];
    % end effector position
    x = x_eb + [rp.l2*cos(th(2));
                rp.l2*sin(th(2))];
            
    J    = Jacobian(th, rp);
    x_d  = J*th_d;
    J_d  = Jacobian_dot(th, th_d, rp);
    x_dd = J*th_dd + J_d*th_d;
end