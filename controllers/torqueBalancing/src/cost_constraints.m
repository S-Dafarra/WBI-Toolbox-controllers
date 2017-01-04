function [hessian,gradient,C,B,Ceq,Beq,Cleq,Bleq,fRH] = cost_constraints(m, Cl, Bl, Cr, Br, ch_points, Pl, Pr, omega, g, ref,gains, gamma0, nsteps, T, k_impact)
%% Third version: new order of chi and first order approximation of angular momentum
%% Inputs
% Mg the mass matrix around the COM
% Cl and Bl are the matrices constraining the left foot wrench with the inequality Cl*fl<=Bl
% Cr and Br the same as above for the right foot
% ch_points are the points delimiting the convex hull on a counter-clockwise order, computed assuming the right foot on the ground
% Pl and Pr are the positions of the left and right foot respectively.
% S_fL and S_fR are the skew-symmetric matrices obtained from fL(0) and fR(0).
% omega is the time constant used for computing the instantaneous capture point
% g is the modulus of the gravity acceleration
% ref is a struct containing the reference for the COM and for the ICP, composed by:
%            -a 9-by-nsteps matrix identifying the reference for gamma (see below) at each time step, ref.COM,
%`           -a 2 rows column vector with the reference for the position of the instantaneous capture point (constant reference), ref.ICP,
%            -a 12-rows coloumn vector with the previous desired wrench, ref.F        
% gains is a struct containing the gains for the cost function, composed by:
%           -a 3*3 matrix with the gains for the COM (coloumnwise)[Kp,Kd,Kw], gains.COM
%           -a 3*3 matrix with the terminal gains for the COM (coloumnwise)[KpT,KdT,KwT], gains.TerCOM
%           -a 2*1 vector containing the gains for the icp [Kicpx;Kicpy], gains.ICP
%           -a 12*2 matrix with the gains proportional to the magnitude of the force and to difference for the desired force from one time step to the next [Kf,Kdf], gains.F
% gamma0 is the initial state [comx;COMvel;w]
% nsteps is the horizon length
% T is the legth in time of the step
% k_impact indicates in which time instant between 1 and nsteps the right
%          foot is in contact with the ground. If 0 the impact is already occurred,
%          if 1 it happens at the immediate next time step (at the beginning of the first step)



%% Extractors

%The state "chi" is composed as follows:
%[gamma(1);f(0);...;gamma(i);f(i-1);...;gamma(nsteps);f(nsteps-1)] where
%gamma(i)=[x_com(i);v_com(i);w_com(i)], the position, linear speed and angular momentum respectively,
%while f(i)=[fl(i);fr(i)] which are the 6D wrech (force and torque, in this order) applied on the left and right foot respectively.

eGamma_cell = repmat({[eye(9),zeros(9,12)]},1,nsteps);
eGamma = sparse(blkdiag(eGamma_cell{:})); %extracts the set of states from chi

eF_cell = repmat({[zeros(12,9),eye(12)]},1,nsteps);
eF =  sparse(blkdiag(eF_cell{:})); %extract the set of wrenches from chi.

eF_l_cell = repmat({[eye(6),zeros(6)]},1,nsteps);
eF_l = sparse(blkdiag(eF_l_cell{:})); %extracts the left foot wrench from the vector of forces

eF_r_cell = repmat({[zeros(6),eye(6)]},1,nsteps);
eF_r = sparse(blkdiag(eF_r_cell{:})); %extracts the right foot wrench from the vector of forces

eICP = sparse([eye(2),zeros(2,1)]*[eye(3),1/omega*eye(3),zeros(3)]); %it computes the instantaneous capture point (just x and y are considered) starting from gamma(i)
eICP_cell = repmat({eICP},1,nsteps);
eICP_hor = sparse(blkdiag(eICP_cell{:})); %extract the icp from the overall vector gamma

%% Model Constraints
%The evolution of the state is as follows:
%gamma(i+1) = Ev_gamma*gamma(i) + F_gamma*f(i) - G_gamma*|g| + S0_gamma with
%g the modulus of the gravity acceleration
%For what concerns the angular momentum, we have w(k+1)=w(k) + T (tau + (Pl-CoMx0) X fl + (Pr-CoMx0) X fr + f_l(0) X comx + f_r(0) X comx - f_l(0) X CoMx0 - f_r(0) X CoMx0)
% which is the Taylor expansion stopped at the first order. The last two
% terms are included into S0_gamma;

CoMx0 = gamma0(1:3);
S_Xl = Sf(Pl-CoMx0);
S_Xr = Sf(Pr-CoMx0);
S_fL = Sf(ref.F(1:3));
S_fR = Sf(ref.F(7:9));

Ev_gamma = [         eye(3),   T*eye(3), zeros(3,3);
                 zeros(3,3),     eye(3), zeros(3,3);
            T*(S_fL + S_fR), zeros(3,3),     eye(3)]; %forward euler

F_gamma = [zeros(3,12);
           T*m^-1*[eye(3),zeros(3),eye(3),zeros(3)];
           T*[S_Xl,eye(3),S_Xr,eye(3)]];
       
G_gamma = [zeros(3,3);T*eye(3);zeros(3)]*[0;0;abs(g)]; %gravity affects just the z direction of the speed evolution

S0_gamma = [zeros(6,1);
            -T*(S_fL+S_fR)*CoMx0];

state_dim =(9+12)*nsteps;

%The state evolves according to this equation: E_gamma*chi = Ev*chi - G + Ev_gamma0 +S0_gamma

Ev_gamma0 = [Ev_gamma;zeros(9*(nsteps-1),9)]*gamma0;

G = repmat(G_gamma,nsteps,1);

S0 = repmat(S0_gamma,nsteps,1);

Ev_gamma_cell = repmat({[Ev_gamma,zeros(9,12)]},1,nsteps-1);
F_gamma_cell = repmat({[zeros(9),F_gamma]},1,nsteps);

%set Ev_gamma_cell on the under diagonal
Ev_gamma_hor = sparse([[zeros(9,21*(nsteps-1));blkdiag(Ev_gamma_cell{:})],zeros(9*nsteps,21)]);

Ev = sparse(blkdiag(F_gamma_cell{:})+Ev_gamma_hor);

%Definition of the constraints matrices 
C_ev = Ev - eGamma;
B_ev = G - Ev_gamma0 - S0;

%% Constraints on the Left Foot

C_horL_cell = repmat({Cl},nsteps,1);
C_horL = sparse(blkdiag(C_horL_cell{:}))*eF_l*eF; %first extracts the wrenches from chi, then the left foot wrenches and finally multiply by Cl
B_horL = repmat(Bl,nsteps,1);

%% Constraints on the Right Foot
ncr = size(Cr,1);  %number of constraints on the right foot
Cr_cell = repmat({Cr},nsteps,1);
Cr_hor = sparse(blkdiag(Cr_cell{:}));

if (k_impact > nsteps)
    C_horR = eF_r*eF; %all the right wrenches are set to zero    
    C_horRI = zeros(0,state_dim);     %just a null matrix, but it has to be defined

else
    if k_impact>1
        C_horR = eF_r(1:(6*(k_impact-1)),1:(12*(k_impact-1)))*eF(1:(12*(k_impact-1)),:); %the rightmost matrix extracts just the first (k_impact-1) wrenches, while the other one selects just the right wrench
        C_horRI = Cr_hor((ncr*(k_impact-1)+1):end,(6*(k_impact-1)+1):end)*eF_r((6*(k_impact-1)+1):end,(12*(k_impact-1)+1):end)*eF((12*(k_impact-1)+1):end,:); %same as above, but the last wrenches are extracted, then multiplied by Cr
    
    else
         C_horR = zeros(0,state_dim);
         C_horRI = Cr_hor*eF_r*eF;
    end
end

B_horR = zeros(size(C_horR,1),1);
B_horRI = repmat(Br,size(C_horRI,1)/ncr,1);

%% Constraints on the Capture Point

[Cch,Bch] = ch_constraints(ch_points); %if Cch*x<=Bch then x is in the convex hull
nch = size(Cch,1);
Cch_cell = repmat({Cch},nsteps,1);
Cch_hor = sparse(blkdiag(Cch_cell{:}));

if (k_impact > nsteps)
    C_horCH = zeros(0,state_dim);
else
    if k_impact>1
        C_horCH = Cch_hor((nch*(k_impact-1)+1):end,(2*(k_impact-1)+1):end)*eICP_hor((2*(k_impact-1)+1):end,(9*(k_impact-1)+1):end)*eGamma((9*(k_impact-1)+1):end,:);
    
    else
        C_horCH = Cch_hor*eICP_hor*eGamma;
    end
end


B_horCH = repmat(Bch,size(C_horCH,1)/nch,1);


%% Overall Constraints: C*chi<=B
%The constraints on the dynamic and on the right foot before the impact are
%with the equal sign, therefore, for example, C_horR*chi<=B_horR and
%-C_horR*chi<=-B_horR.

C = [C_ev;  
     -C_ev;
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
      
 Ceq = [C_ev;
        C_horR];
    
 Beq = [B_ev;
        B_horR];
    
 Cleq = [C_horL;
         C_horRI;
         C_horCH];
              
 Bleq = [B_horL;
         B_horRI;
         B_horCH];
          
 %% Definition of the cost function
 
%Sum(over all steps)[0.5*K_gamma(gamma-gamma_desired)^2 + 0.5*Kf*(f)^2 + 0.5*Kdf*(f-f(i-1))^2] + Sum(over all the steps after the impact)[0.5*k_icp*(icp-icp_desired)^2]

%% Cost related to gamma

gamma_d = sparse(reshape(ref.COM,[],1));

K_hor_gamma = sparse(diag(repmat(reshape(gains.COM,[],1),nsteps,1)));
hes_gamma = eGamma'*K_hor_gamma*eGamma;
grad_gamma = -eGamma'*K_hor_gamma*gamma_d;

%% Terminal cost on gamma

K_hor_gammaTer = sparse(diag([zeros(9*(min(nsteps,k_impact)-1),1);repmat(reshape(gains.TerCOM,[],1),nsteps-min(nsteps,k_impact)+1,1)]));
hes_gamma_Ter = eGamma'*K_hor_gammaTer*eGamma;
grad_gamma_Ter = -eGamma'*K_hor_gammaTer*gamma_d;

%% Cost related to the icp

if (k_impact > nsteps)
    K_icp_hor = sparse(zeros(2*nsteps));
else
    if k_impact>0
        K_icp_hor = sparse(blkdiag( zeros(2*(k_impact-1)), diag(repmat(gains.ICP,nsteps-k_impact+1,1))));    
    else
        K_icp_hor = sparse(diag(repmat(gains.ICP,nsteps,1)));
    end
end

hes_icp = eGamma'*eICP_hor'*K_icp_hor*eICP_hor*eGamma;
grad_icp = -eGamma'*eICP_hor'*K_icp_hor*repmat(ref.ICP,nsteps,1);

%% Cost related to forces

Kf_hor = sparse(diag(repmat(gains.F(:,1),nsteps,1)));

hes_f = eF'*Kf_hor*eF;

Fdif = sparse(eye(12*nsteps)-diag(ones(12*(nsteps-1),1),-12));
Kdf_hor = sparse(diag(repmat(gains.F(:,2),nsteps,1)));
f0 = sparse([ref.F;zeros(12*nsteps-length(ref.F),1)]);

hes_df = eF'*Fdif'*Kdf_hor*Fdif*eF;
grad_df = -eF'*Fdif'*Kdf_hor*f0;

%% Overall cost
hessian = hes_gamma + hes_icp + hes_f + hes_df + hes_gamma_Ter;
gradient = grad_gamma + grad_icp + grad_df + grad_gamma_Ter;

%% Extract f(0)
fRH = eF(1:12,:);
