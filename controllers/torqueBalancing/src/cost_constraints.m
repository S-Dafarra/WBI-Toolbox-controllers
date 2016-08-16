function [hessian,gradient,C,B,time] = cost_constraints(Mg, Cl, Bl, Cr, Br, ch_points, Alr, omega, g, ref,gains, gamma0, nsteps, T, k_impact)
%Inputs
% Mg the mass matrix around the COM
% Cl and Bl are the matrices constraining the left foot wrench with the inequality Cl*fl<=Bl
% Cr and Br the same as above for the right foot
% ch_points are the points delimiting the convex hull on a counter-clockwise order, computed assuming the right foot on the ground
% Alr is the matrix projecting the right and left wrench on the COM
% omega is the time constant used for computing the instantaneous capture point
% g is the modulus of the gravity acceleration
% ref is a struct containing the reference for the COM and for the ICP, composed by:
%            -a 9-by-nsteps matrix identifying the reference for gamma (see below) at each time step, ref.COM,
%`           -a 2 rows column vector with the reference for the position of the instantaneous capture point (constant reference), ref.ICP,
%            -a 6-rows coloumn vector with the previous desired wrench (just on the left foot, since the right foot is assumed to be raised), ref.F
% gains is a struct containing the gains for the cost function, composed by:
%           -a 3*3 matrix with the gains for the COM (coloumnwise)[Kp,Kd,Kw], gains.COM
%           -a 2*1 vector containing the gains for the icp [Kicpx;Kicpy], gains.ICP
%           -a 12*2 matrix with the gains proportional to the magnitude of the force and to difference for the desired force from one time step to the next [Kf,Kdf], gains.F
% gamma0 is the initial state [comx;COMvel;w]
% nsteps is the horizon length
% T is the legth in time of the step
% k_impact indicates in which time instant between 1 and nsteps the right
%          foot is in contact with the ground. If 0 the impact is already occurred,
%          if 1 it happens at the immediate next time step (at the beginning of the first step)

%% Constraints preliminaries
t_prelim = tic;
m = Mg(1,1); %mass of the robot
J_com = Mg(4:6,4:6);
M_inv = blkdiag(eye(3)/m,inv(J_com));

%The state "chi" is composed as follows:
%[gamma(1);gamma(i);gamma(nsteps);f(0);f(i);f(nsteps-1)] where
%gamma(i)=[x_com(i);v_com(i);w_com(i)], the position, linear and angular speed respectively,
%while f(i)=[fl(i);fr(i)] which are the 6D wrech applied on the left and right foot respectively.

%The evolution of the state is as follows:
%gamma(i+1) = Ev_gamma*gamma(i) + F_gamma*f(i) - G_gamma*|g| with
%g the modulus of the gravity acceleration

Ev_gamma = [   eye(3),    T*eye(3), zeros(3,3);
            zeros(3,3),     eye(3), zeros(3,3);
            zeros(3,3), zeros(3,3),     eye(3)]; %forward euler

F_gamma = [zeros(3,12);
           T*M_inv*Alr];
       
G_gamma = [zeros(3,3);T*eye(3);zeros(3)]*[0;0;abs(g)]; %gravity affects just the z direction of the speed evolution

state_dim =(9+12)*nsteps;

%The state evolves according to this equation: chi = Ev*chi - G + Ev_gamma0

Ev_gamma0 = [Ev_gamma;zeros(state_dim-9,9)]*gamma0; %gamma(1) is the only one which depends from the initial state

G = [repmat(G_gamma,nsteps,1);zeros(12*nsteps,1)]; %just gamma is affected by the gravity, not the forces

Ev = zeros(state_dim);

Ev(1:9,:)=[zeros(9,9*nsteps),F_gamma,zeros(9,(nsteps-1)*12)]; %first step of integration
Ev((9*nsteps +1):end,:) = [zeros(12*nsteps,9*nsteps),eye(12*nsteps)]; %forces are equal to themselves

ncl = size(Cl,1); %number of constraints on the left foot
C_horL = zeros(nsteps*ncl,state_dim); %initialization of constraints matrix on the left foot over the horizon

ncr = size(Cr,1);  %number of constraints on the right foot
if (k_impact > nsteps)
    C_horR = zeros(nsteps*6,state_dim); %initialization for the constraint matrix on the right foot for the time instants before the impact 
    C_horRI = zeros(0,state_dim);     %just a null matrix, but it has to be defined
else if k_impact>0
        C_horR = zeros(6*(k_impact-1),state_dim);
        C_horRI = zeros(ncr*(nsteps-k_impact+1),state_dim); %initialization for the constraint matrix on the right foot for the time instants AFTER the impact 
    else C_horR = zeros(0,state_dim);
         C_horRI = zeros(nsteps*ncr,state_dim);
    end
end

[Cch,Bch] = ch_constraints(ch_points); %if Cch*x<=Bch then x is in the convex hull
nch = size(Cch,1);
if (k_impact > nsteps)
    C_horCH = zeros(0,state_dim);
else if k_impact>0
        C_horCH = zeros(nch*(nsteps-k_impact+1),state_dim);
    else C_horCH = zeros(nsteps*nch,state_dim);
    end
end

el_prelim = toc(t_prelim);
%% Computation of constraints for all the time steps
t_for = tic;

for i = 1:nsteps
   %% Constraints introduced by the dynamics
    if (i>1) % the evolution for i=1 has been already defined above
        Ev((9*(i-1)+1):(9*i),:) = [zeros(9,9*(i-2)),Ev_gamma,zeros(9,9*(nsteps-i+1)),zeros(9,12*(i-1)),F_gamma,zeros(9,12*(nsteps-i))];
    end
    
    %% Constraints for the left foot wrench
    C_horL((ncl*(i-1)+1):(ncl*i),:) = full(sparse(Cl)*sparse([eye(6),zeros(6)])*sparse([zeros(12,9*nsteps),zeros(12,12*(i-1)),eye(12),zeros(12,12*(nsteps-i))]));
    %For each time instant, Cl is multiplied by a matrix that extracts the
    %left foot, than by a matrix which extracts the left and ri[ht wrench
    %from the overall state vector
    
    %% Constraints for the right foot wrench
    if (i<k_impact)
         C_horR((6*(i-1)+1):(6*i),:) = full(sparse([zeros(6,6),eye(6)])*sparse([zeros(12,9*nsteps),zeros(12,12*(i-1)),eye(12),zeros(12,12*(nsteps-i))]));
    else C_horRI((ncr*(i-k_impact)+1):(ncr*(i-k_impact+1)),:) = full(sparse(Cr)*sparse([zeros(6,6),eye(6)])*sparse([zeros(12,9*nsteps),zeros(12,12*(i-1)),eye(12),zeros(12,12*(nsteps-i))]));
    end
    %The wrench on the right foot should be zero before the impact and
    %satisfing the same constraints of the left foot after the impact;
    %% Constraints on the icp
    if (i>=k_impact)
        C_horCH((nch*(i-k_impact)+1):(nch*(i-k_impact+1)),:) = full(sparse([Cch,zeros(nch,1)])*sparse([eye(3),1/omega*eye(3),zeros(3)])*sparse([zeros(9,9*(i-1)),eye(9),zeros(9,9*(nsteps-i)),zeros(9,12*nsteps)]));
        % the first matrix avoid considering the z component, the second
        % computes the instantaneous capture point from the state of the
        % COM extracted by the third matrix. These constraints impose the
        % instantaneous capture point in the convex hull after the step is
        % performed.
    end
   
end
el_for = toc(t_for);
t_B = tic;
%% Definition of the B matrices
B_ev = G-Ev_gamma0;
B_horL = repmat(Bl,nsteps,1);
B_horR = zeros(size(C_horR,1),1);
B_horRI = repmat(Br,size(C_horRI,1)/ncr,1);
B_horCH = repmat(Bch,size(C_horCH,1)/nch,1);

%% Overall Constraints: C*chi<=B
%The constraints on the dynamic and on the right foot before the impact are
%with the equal sign, therefore, for example, C_horR*chi<=B_horR and
%-C_horR*chi<=-B_horR.

C = [Ev-eye(state_dim);
     eye(state_dim)-Ev;
     C_horL;
     C_horR;
     -C_horR;
     C_horRI;
     C_horCH];
 
 B = [B_ev;
     -B_ev;
     B_horL;
     B_horR;
     -B_horR;
     B_horRI;
     B_horCH];
 
 
 el_B = toc(t_B);
 %% Definition of the cost function
%Sum(over all steps)[0.5*K_gamma(gamma-gamma_desired)^2 + 0.5*Kf*(f)^2 + 0.5*Kdf*(f-f(i-1))^2] + Sum(over all the steps after the impact)[0.5*k_icp*(icp-icp_desired)^2]
t_cost = tic;
%% Cost related to gamma
K_gamma = sparse(diag(reshape(gains.COM,[],1)));
gamma_d = sparse(reshape(ref.COM,[],1));

eGAMMA = sparse([eye(9*nsteps),zeros(9*nsteps,12*nsteps)]); %extract the set of gamma from chi

K_gamma_cell = repmat({K_gamma},1,nsteps);
K_hor_gamma = sparse(blkdiag(K_gamma_cell{:})); %repeat K_gamma nsteps time along the diagonal.
hes_gamma = eGAMMA'*K_hor_gamma*eGAMMA;
grad_gamma = -eGAMMA'*K_hor_gamma*gamma_d;

%% Cost related to the icp

e_ICP = sparse([eye(2),zeros(2,1)]*[eye(3),1/omega*eye(3),zeros(3)]); %it computes the instantaneous capture point (just x and y are considered) starting from gamma(i)
e_ICP_cell = repmat({e_ICP},1,nsteps);
e_ICP_hor = sparse(blkdiag(e_ICP_cell{:})); %extract the icp from the overall vector gamma


if (k_impact > nsteps)
    K_icp_hor = sparse(zeros(2*nsteps));
else if k_impact>0
        K_icp_hor = sparse(blkdiag( zeros(2*(k_impact-1)), diag(repmat(gains.ICP,nsteps-k_impact+1,1))));    
    else K_icp_hor = sparse(diag(repmat(gains.ICP,nsteps,1)));
    end
end

hes_icp = eGAMMA'*e_ICP_hor'*K_icp_hor*e_ICP_hor*eGAMMA;
grad_icp = -eGAMMA'*e_ICP_hor'*K_icp_hor*repmat(ref.ICP,nsteps,1);

%% Cost related to forces

eF = sparse([zeros(12*nsteps,9*nsteps),eye(12*nsteps)]);    %extract the set of wrenches from chi
Kf_hor = sparse(diag(repmat(gains.F(:,1),nsteps,1)));

hes_f = eF'*Kf_hor*eF;

Fdif = sparse(eye(12*nsteps)-diag(ones(12*(nsteps-1),1),-12));
Kdf_hor = sparse(diag(repmat(gains.F(:,2),nsteps,1)));
f0 = [ref.F;zeros(12*nsteps-length(ref.F),1)];

hes_df = eF'*Fdif'*Kdf_hor*Fdif*eF;
grad_df = -eF'*Fdif'*Kdf_hor*f0;

%% Overall cost
hessian = full(hes_gamma + hes_icp + hes_f + hes_df);
gradient = full(grad_gamma + grad_icp + grad_df);


el_cost = toc(t_cost);
time = [el_prelim;el_for;el_B;el_cost];

