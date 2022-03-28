function [te,xe,t,x] = pendulum_dynamics(t, x0, TD, par)
    x = ode2(@robot_stand_eom,t,x0,par);
    te = t(end);
    xe = x(end,:);

    function xdot = robot_stand_eom(t,x,par)
        p  = x(1);
        pd = x(2);

        M = diag([par.m1, par.m1, par.I1, par.m2, par.m2, par.I2, par.m3, par.m3, par.I3, par.m4, par.m4, par.I4]);
        c = par.c4;

        T =[      0
                  0
                  0
                  0
                  0
                  0
                  0
                  0
                  0
         c*cos(p)
         c*sin(p)
                  1];



        TT = [           0
                         0
                         0
                         0
                         0
                         0
                         0
                         0
                         0
         -c*sin(p)*pd^2
          c*cos(p)*pd^2
                         0];

        % construction of reduced mass matrix  
        Mr = T.'*M*T;

        %gravity forces
        fg = M * [ 0;
                  -par.g;
                   0;
                   0;
                  -par.g;
                   0;
                   0;
                  -par.g;
                   0;
                   0;
                  -par.g;
                   0];

        % reduced force vector
        fr = T.'*fg  - T.'*M*TT + -TD;

        % solve equations of motion
        qdd = Mr\fr;

        % the derivative of the state consists of the (known) velocities and qdd
        xdot = [pd; qdd];
    end
end


 