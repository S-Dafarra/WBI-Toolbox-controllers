
nsteps = 15;
T= 0.01;
k_impact = 0;


%Mg = blkdiag(diag(30*ones(3,1)),rand(3,3))
Cl = ConstraintsMatrix;
Cr = Cl;
Bl = bVectorConstraints;
Br = Bl;
wHl = [eye(3), [0;0;0];
       zeros(1,3),1];
Pl = wHl(1:3,4);
   
theta = -30/180*pi;

wHr = [cos(theta), -sin(theta), 0,  0.05;
       sin(theta),  cos(theta), 0,   0.3;
       0,                    0, 1,     0;
       0,                    0, 0,     1];
   
Pr = wHr(1:3,4);

foot_size =  [ -0.05   0.10 ;    
               -0.02   0.02];
      
xm = foot_size(1,1);
xM = foot_size(1,2);
ym = foot_size(2,1);
yM = foot_size(2,2);

pointsl = wHl*[xM, xM, xm, xm;
               ym, yM, yM, ym;
                0,  0,  0,  0;
                1,  1,  1,  1];

pointsr = wHr*[xM, xM, xm, xm;
               ym, yM, yM, ym;
               0,  0,  0,  0;
               1,  1,  1,  1];
             
xch = [pointsl(1,:),pointsr(1,:)]; %consider just the projection on the ground
ych = [pointsl(2,:),pointsr(2,:)]; 

K_in = zeros(length(xch),1);
coder.extrinsic('mpc_ch')
K_in = mpc_ch(xch,ych);
K = K_in(1:sum(K_in>0)); %avoid considering the zeros used to fill the vector
ch_points = [xch(K)',ych(K)'];
omega = 9.81/2;
g = 9.81;
COMx = ones(3,1);
COMv = zeros(3,1);
COMdes = [0.5*ones(3,1),zeros(3,1)];
f_prev = [0;0;300;0;0;0;zeros(6,1)];
gains.COM = zeros(3);
gains.TerCOM = 1000*rand(3);
gains.ICP = zeros(2,1);
gains.F = [ones(12,1),1*ones(12,1)];
gamma0 = [0.5*ones(3,1);zeros(6,1)];
m =30;
minZ = 0;
ICPoffset = [0;0];

%[hessian,gradient,C,B,Ceq,Beq,Cleq,Bleq,fRH,time] = cost_constraints(m, Cl, Bl, Cr, Br, ch_points, Alr, omega, g, ref,gains, gamma0, nsteps, T, k_impact);
% tic
% [chi,~,exitFlagQP,~,~,~] = qpOASES(hessian,gradient,C,[],[],[],B);
% toc
% fRH*chi
tic
[f,COM_des,exit_flag,~,chi, COM_last] = solve_mpc_cvx(m, Cl, Bl, Cr, Br, Pl, Pr, omega, g, f_prev, COMx, COMv, COMdes, ICPoffset, minZ, gains, gamma0, nsteps, T, k_impact); 
f
exit_flag
COM_last
toc

% AL              = [ eye(3),zeros(3);
%                     Sf(Pl),  eye(3)];
% AR              = [ eye(3), zeros(3);
%                      Sf(Pr), eye(3) ];
%                  
% Alr               = [ AL, AR];
% tic
% [f2,COM_des2,exitflag2,~,chi2] = solve_mpc_cvxV1(m, Cl, Bl, Cr, Br, [xch(K)',ych(K)'], Alr, omega, 9.81, f_prev, COMx, COMv, COMdes, [0;0], minZ, gains, gamma0, nsteps, T, k_impact); 
% %f
% exitflag2
% disp('New method');
% toc

% 
% k_impact = k_impact-1;
% tic
% Ev_gamma = [   eye(3),    T*eye(3), zeros(3,3);
%             zeros(3,3),     eye(3), zeros(3,3);
%             zeros(3,3), zeros(3,3),     eye(3)]; %forward euler
% 
% F_gamma = [zeros(3,12);
%            T*blkdiag(m^-1*eye(3),eye(3))*Alr];
%        
% G_gamma = [zeros(3,3);T*eye(3);zeros(3)]*[0;0;9.81];
% 
% new_gamma = Ev_gamma*chi(9*(nsteps-1)+1:9*nsteps) + F_gamma*chi((length(chi)-11):end) - G_gamma;
% x0 = [chi(10:9*(nsteps-1));
%       new_gamma;
%       chi((9*nsteps+12):end);
%       chi((length(chi)-12):end)];
% 
% [COM_hor_des,COMv_hor_des] = fourth_traj(COMx(1:3),COMdes(1:3,2),COMdes(1:3,1),zeros(3,1),zeros(3,1),T,1,nsteps); %bypassed for the moment
% 
% ref.COM = [COM_hor_des(1:3,1:nsteps);
%            COMv_hor_des(1:3,1:nsteps);
%            zeros(3,nsteps)];
% 
% if(sum(ch_points(1,:)==ch_points(end,:))==2) %the last point is equal to the first
%      ref.ICP = mean(ch_points(1:(end-1),:))'; %the centroid of the convex hull
% else ref.ICP = mean(ch_points)'; %the centroid of the convex hull
% end
% 
% ref.F = f_prev;
% 
% [hessian,gradient,C,B,Ceq,Beq,Cleq,Bleq,fRH,~] = cost_constraints(m, Cl, Bl, Cr, Br, ch_points, Alr, omega, g, ref, gains, gamma0, nsteps, T, k_impact);
% 
% %size([full(hessian),full(Ceq');full(Ceq),zeros(size(full(Ceq),1))])
% %rank([full(hessian),full(Ceq');full(Ceq),zeros(size(full(Ceq),1))])
% 
% f=zeros(12,1);
% COM_des = zeros(9,1);
% 
% % options = optimoptions('quadprog','Algorithm','interior-point-convex','OptimalityTolerance',1e-3,'Display','off');
% % [chi,~,exitflag] = quadprog(hessian,gradient,Cleq,Bleq,Ceq,Beq,[],[],x0,options);
% % hessian = full(hessian);
% % gradient = full(gradient);
% % C = full(C);
% % B = full(B);
% [chi,~,exitFlagQP,~,~,~] = qpOASES(hessian,gradient,C,[],[],[],B);
% 
% if (exitflag>0)
%     f=fRH*chi;
%     COM_des = chi(1:9);
% end
% 
% COMref = COM_hor_des(1:3,1);
% toc
  
% ref.minZ = minZ;
% 
% ref.COM = sym('COM_hor',[9 nsteps]);
% 
% ref.ICP = sym('ICPref',[2,1]);
% 
% ref.F = sym('F_prev',[12 1]);
% 
% m = sym('m');
% 
% Cl = sym('Cl',size(ConstraintsMatrix));
% 
% Cr = sym('Cr',size(ConstraintsMatrix));
% 
% Alr = sym('Alr',[6,12]);
% 
% omega = sym('omega');
% 
% gamma0 = sym('gamma0',[9,1]);
% 
% Cch = sym('Cch',[6,2]);
% Bch = sym('Bch',[6,1]);
% 
% tic
% [hessian,gradient,~,~,Ceq,Beq,Cleq,Bleq,fRH,~] = cost_constraints_sym(m, Cl, Bl, Cr, Br, Cch, Bch, Alr, omega, 9.81, ref, gains, gamma0, nsteps, T, k_impact);


