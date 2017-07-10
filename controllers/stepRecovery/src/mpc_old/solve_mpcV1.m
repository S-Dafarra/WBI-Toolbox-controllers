function [f,COM_des,exitflag,COMref,chi] = solve_mpcV1(m, Cl, Bl, Cr, Br, ch_points, Alr, omega, g, f_prev, COMx,COMv,COMdes, ICPoffset, minZ, gains, gamma0, nsteps, dT, k_impact)

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

[hessian,gradient,~,~,Ceq,Beq,Cleq,Bleq,fRH,~] = cost_constraintsV1(m, Cl, Bl, Cr, Br, ch_points, Alr, omega, g, ref, gains, gamma0, nsteps, dT, k_impact);

%size([full(hessian),full(Ceq');full(Ceq),zeros(size(full(Ceq),1))])
%rank([full(hessian),full(Ceq');full(Ceq),zeros(size(full(Ceq),1))])

f=zeros(12,1);
COM_des = zeros(9,1);

options = optimoptions('quadprog','Algorithm','interior-point-convex','OptimalityTolerance',1e-3,'Display','off');
[chi,~,exitflag] = quadprog(hessian,gradient,Cleq,Bleq,Ceq,Beq,[],[],[],options);

if (exitflag>0)
    f=fRH*chi;
    COM_des = chi(1:9);
end

COMref = COM_hor_des(1:3,1);
 
end