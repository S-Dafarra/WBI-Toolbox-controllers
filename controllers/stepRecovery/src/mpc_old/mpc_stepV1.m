function [f,exitflag,COM_des_out,COMref] = mpc_stepV1(mpc_init,constraints_vec, ConstraintsMatrix, bVectorConstraints, w_H_l_sole,lsole_H_r_sole, w_H_r_sole_in, COMx, COMv, Hw, COMdes, minZ, f_prev, foot_size, omega, m, dT, STEP)

persistent elapsed_time;
persistent COMref_prev;


nsteps = mpc_init.nsteps;
gains = mpc_init.gains;
step_time = mpc_init.tstep;

if isempty(elapsed_time)
    elapsed_time = 0;
end


if(constraints_vec(2)>0)
    k_impact = 1; %%impact already happened
    elapsed_time = 0;
else if STEP
        k_impact = max(ceil((step_time - elapsed_time)/dT),2); %basically the 2 avoids requiring wrench on the right foot for a wrong preview on the step time
        if (elapsed_time + dT) <= step_time
            elapsed_time = elapsed_time + dT;
        end
    else k_impact = nsteps +1;
         elapsed_time = 0;
    end
end  


if constraints_vec(2)>0
        w_H_r_sole = w_H_r_sole_in;
else
        remove_z = zeros(4);
        remove_z(3,4)= lsole_H_r_sole(3,4);
        w_H_r_sole = w_H_l_sole*(lsole_H_r_sole - remove_z); %imposes right and left foot to be at the same eight.
end

pos_leftFoot   = w_H_l_sole(1:3,4);
w_R_l_sole     = w_H_l_sole(1:3,1:3);

pos_rightFoot   = w_H_r_sole(1:3,4);
w_R_r_sole      = w_H_r_sole(1:3,1:3);

Pr              = pos_rightFoot - COMx;
Pl              = pos_leftFoot  - COMx;

AL              = [ eye(3),zeros(3);
                    Sf(Pl),  eye(3)];
AR              = [ eye(3), zeros(3);
                     Sf(Pr), eye(3) ];
                 
Alr               = [ AL, AR];

Cl  = ConstraintsMatrix * blkdiag(w_R_l_sole',w_R_l_sole');
Cr = ConstraintsMatrix * blkdiag(w_R_r_sole',w_R_r_sole');
Bl = bVectorConstraints;
Br = bVectorConstraints;

    
xm = foot_size(1,1);
xM = foot_size(1,2);
ym = foot_size(2,1);
yM = foot_size(2,2);

pointsl = w_H_l_sole*[xM, xM, xm, xm;
                     ym, yM, yM, ym;
                     0,  0,  0,  0;
                     1,  1,  1,  1];

pointsr = w_H_r_sole*[xM, xM, xm, xm;
                     ym, yM, yM, ym;
                      0,  0,  0,  0;
                      1,  1,  1,  1];
             
xch = [pointsl(1,:),pointsr(1,:)]; %consider just the projection on the ground
ych = [pointsl(2,:),pointsr(2,:)]; 

K_in = zeros(length(xch),1);
coder.extrinsic('mpc_ch')
K_in(:) = mpc_ch(xch,ych);
K = K_in(1:sum(K_in>0)); %avoid considering the zeros used to fill the vector

ch_points = [xch(K)',ych(K)'];

gamma0 = [COMx;COMv;Hw];


COMdesfilt_pos = [(pos_leftFoot(1:2) + pos_rightFoot(1:2))/2;COMdes(3,1)]+ mpc_init.COMoffset;
 
COMdesfilt = [COMdesfilt_pos,zeros(3,1)];

if isempty(COMref_prev) || (constraints_vec(2)==0)
    COMref_prev = COMdesfilt_pos;
end

f= zeros(12,1);
exitflag = 1;
COM_des_out = zeros(9,1);
COMref = zeros(3,1);

coder.extrinsic('solve_mpc_cvxV1')
[f(:),COM_des_out(:,:),exitflag(:),COMref(:,:)] = solve_mpc_cvxV1(m, Cl, Bl, Cr, Br, ch_points, Alr, omega, 9.81, f_prev, COMref_prev, zeros(3,1), COMdesfilt, mpc_init.COMoffset(1:2),minZ, gains, gamma0, nsteps, dT, k_impact); 
COMref_prev = COMref;
