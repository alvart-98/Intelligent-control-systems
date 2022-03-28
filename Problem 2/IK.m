%% ANALYTICAL INVERSE KINEMATICS
% as this is not unique (2pi wrap around) return the closest value to the current theta
function [th, th_d, th_dd] = IK(x, x_d, x_dd, th_curr, rp)
    th = zeros(2,1);
    A = x(1)^2+x(2)^2+rp.l1^2-rp.l2^2+2*rp.l1*x(1);
    B = -4*rp.l1*x(2);
    C = x(1)^2+x(2)^2+rp.l1^2-rp.l2^2-2*rp.l1*x(1);
    
    % catch Cartesian points that are not reachable
    if (B^2-4*A*C)>=0
        th(1) = 2*atan2(-B+sqrt(B^2-4*A*C),2*A);
        th(2) = atan2(x(2)-rp.l1*sin(th(1)),x(1)-rp.l1*cos(th(1)));
    else
        fprintf('Cartesian position not reachable\n');
        th = th_curr;
    end
    
    th_diff = mod((th - th_curr) + pi, 2*pi) - pi; % difference of th to th_curr mapped to [-pi, pi]
    th = th_curr + th_diff;
    
    J = Jacobian(th, rp);
    
    % and a check of whether J is singular
    if rcond(J)>0
        th_d  = J\x_d; % angular velocity
        J_d   = Jacobian_dot(th, th_d, rp);
        th_dd = J\(x_dd - J_d*th_d); % angular accelerations
    else
        fprintf('Jacobian singular\n');
        th_d  = [0; 0];
        th_dd = [0; 0];
    end
end