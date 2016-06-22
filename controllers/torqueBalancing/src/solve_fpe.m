function theta_bar = solve_fpe(theta_dot, m, h, vx, h_dot, J, g)
options = optimoptions(@fsolve,'SpecifyObjectiveGradient',true,'Display','off');
theta_bar = fsolve(@(theta)fpe_residual(theta, theta_dot, m, h, vx, h_dot, J, g),1,options);