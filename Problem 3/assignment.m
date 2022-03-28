function assignment

    par = robot_set_parameters;
    par.run_type = 'learn';
    par = swingup(par);
    
    par.run_type = 'test';
    [par, ta, xa] = swingup(par);
    
    animate_swingup(ta, xa, par)
end
