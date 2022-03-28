function des = calculate_trajectory(t, tp, rp)
    %% Cartesian trajectory
    R1=tp.rx; R2=tp.ry; r=tp.ell_an; w=tp.w;

    des.x = [tp.x0 + R1*cos(w*t)*cos(r)-R2*sin(w*t)*sin(r);
             tp.y0 + R1*cos(w*t)*sin(r)+R2*sin(w*t)*cos(r)];

    des.x_d = [-R1*w*sin(w*t)*cos(r)-R2*w*cos(w*t)*sin(r);
               -R1*w*sin(w*t)*sin(r)+R2*w*cos(w*t)*cos(r)];

    des.x_dd = [-R1*(w^2)*cos(w*t)*cos(r)+R2*(w^2)*sin(w*t)*sin(r);
                -R1*(w^2)*cos(w*t)*sin(r)-R2*(w^2)*sin(w*t)*cos(r)];

    %% Joint trajectory
    
    des.th    = zeros(2,length(t)+1); % des.th is for now padded with a zero vector on index 1
    des.th_d  = zeros(size(des.th));
    des.th_dd = zeros(size(des.th));
    
    for iter = 1:length(t)
        % IK of desired Cartesian position [as close as possible to last
        % joint position des.th(:,iter-1) ]
        
        [des.th(:,iter+1), des.th_d(:,iter+1), des.th_dd(:,iter+1)] = ...
            IK(des.x(:,iter), des.x_d(:,iter), des.x_dd(:,iter), des.th(:,iter), rp);
    end
    
    % remove padding
    des.th = des.th(:,2:end); des.th_d = des.th_d(:,2:end); des.th_dd = des.th_dd(:,2:end);
end