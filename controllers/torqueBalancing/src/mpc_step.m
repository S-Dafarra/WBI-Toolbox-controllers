function [f,exitflag,COM_des] = mpc_step(mpc_init,constraints_vec, ConstraintsMatrix, bVectorConstraints, w_H_l_sole,lsole_H_r_sole, w_H_r_sole_in, COMx, COMv, Hw, COMdes, f_prev, foot_size, omega, m, dT, STEP)

persistent elapsed_time;


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
        k_impact = min(ceil((step_time - elapsed_time)/dT),2); %basically the 2 avoids requiring wrench on the right foot for a wrong preview on the step time
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
K_in = mpc_ch(xch,ych);
K = K_in(1:sum(K_in>0)); %avoid considering the zeros used to fill the vector

gamma0 = [COMx;COMv;Hw];

f= zeros(12,1);
exitflag = 1;
COM_des = zeros(9,1);

coder.extrinsic('solve_mpc')
[f,COM_des,exitflag] = solve_mpc(m, Cl, Bl, Cr, Br, [xch(K)',ych(K)'], Alr, omega, 9.81, f_prev, COMx, COMv, COMdes, gains, gamma0, nsteps, dT, k_impact); 

