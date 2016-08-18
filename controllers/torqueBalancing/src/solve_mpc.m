function [f,COM_des,exitflag] = solve_mpc(m, Cl, Bl, Cr, Br, ch_points, Alr, omega, g, f_prev, COMx,COMv,COMdes, gains, gamma0, nsteps, dT, k_impact)



[COM_hor_des,COMv_hor_des] = fourth_traj(COMx,COMv,COMdes,zeros(3,1),zeros(3,1),dT,max(nsteps,k_impact));

ref.COM = [COM_hor_des(1:nsteps);
           COMv_hor_des(1:nsteps);
           zeros(3,nsteps)];
    
ref.ICP = mean(ch_points)'; %the centroid of the convex hull

ref.F = f_prev;


[hessian,gradient,~,~,Ceq,Beq,Cleq,Bleq,fRH,~] = cost_constraints(m, Cl, Bl, Cr, Br, ch_points, Alr, omega, g, ref, gains, gamma0, nsteps, dT, k_impact);

options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
[chi,~,exitflag] = quadprog(hessian,gradient,Cleq,Bleq,Ceq,Beq,[],[],[],options);

f=fRH*chi;
COM_des = chi(1:9);
end