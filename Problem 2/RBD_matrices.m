function [M, V, G] = RBD_matrices(theta,rp)
    c1=cos(theta(1));
    c2=cos(theta(2));
    c21=cos(theta(2)-theta(1));
    s21=sin(theta(2)-theta(1));

    %% Dynamics matrices
    M = [rp.j1+rp.m2*rp.l1^2,    rp.m2*rp.l1*rp.lc2*c21;
         rp.m2*rp.l1*rp.lc2*c21, rp.j2+rp.m2*rp.lc2^2  ];
    V = [0,                      -rp.m2*rp.l1*rp.lc2*s21;
        rp.m2*rp.l1*rp.lc2*s21,  0                     ];
    G = [rp.m1*rp.g*rp.lc1*c1+rp.m2*rp.g*rp.l1*c1;
        rp.m2*rp.g*rp.lc2*c2];
end