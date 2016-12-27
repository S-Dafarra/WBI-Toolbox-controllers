function [f,COM_des,exit_flag,COMref,chi] = solve_mpc_cvx(m, Cl, Bl, Cr, Br, ch_points, Pl, Pr, omega, g, f_prev, COMx,COMv,COMdes, ICPoffset, minZ, gains, gamma0, nsteps, dT, k_impact)

ref.minZ = minZ;

[COM_hor_des,COMv_hor_des] = fourth_traj(COMx(1:3),COMdes(1:3,2)*0,COMdes(1:3,1),zeros(3,1),zeros(3,1),dT,1*0+100,nsteps); %bypassed for the moment

ref.COM = [COM_hor_des(1:3,1:nsteps);
           COMv_hor_des(1:3,1:nsteps);
           zeros(3,nsteps)];

if(sum(ch_points(1,:)==ch_points(end,:))==2) %the last point is equal to the first
     ref.ICP = mean(ch_points(1:(end-1),:))'+ICPoffset; %the centroid of the convex hull
else ref.ICP = mean(ch_points)'+ICPoffset; %the centroid of the convex hull
end

ref.F = f_prev;

[hessian,gradient,~,~,Ceq,Beq,Cleq,Bleq,fRH] = cost_constraints(m, Cl, Bl, Cr, Br, ch_points, Pl, Pr,omega, g, ref, gains, gamma0, nsteps, dT, k_impact);

%size([full(hessian),full(Ceq');full(Ceq),zeros(size(full(Ceq),1))])
%rank([full(hessian),full(Ceq');full(Ceq),zeros(size(full(Ceq),1))])

f=zeros(12,1);
COM_des = zeros(9,1);

% options = optimoptions('quadprog','Algorithm','interior-point-convex','OptimalityTolerance',1e-3,'Display','off');
% [chi,~,exitflag] = quadprog(hessian,gradient,Cleq,Bleq,Ceq,Beq,[],[],[],options);
n = 21*nsteps;
precision = cvx_precision('low');
cvx_begin quiet
cvx_solver mosek
cvx_precision(precision);
    variable chi(n);
    minimize (0.5*chi'*hessian*chi + chi'*gradient);
    subject to
              Ceq*chi == Beq;
              Cleq*chi <= Bleq;
cvx_end
exit_message = cvx_status;
if (strcmp(exit_message,'Solved')||strcmp(exit_message,'Inaccurate/Solved'))
    exit_flag = 1;
    f=fRH*chi;
    COM_des = chi(1:9);
else
    exit_flag = 0;
end

COMref = COM_hor_des(1:3,1);
 
end