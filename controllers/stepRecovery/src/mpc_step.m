function [f,exitflag,COM_des_out,COMref, MPC_STEP, errorLastCom] = mpc_step(mpc_init,constraints_vec, ConstraintsMatrix, bVectorConstraints, w_H_l_sole,lsole_H_r_sole, w_H_r_sole_in, COMx, COMv, Hw, COMdesStateMachine, minZ, f_prev, foot_size, omega, m, dT, state)

persistent elapsed_time;
persistent COMref_prev;


nsteps = mpc_init.nsteps;
gains = mpc_init.gains;
step_time = mpc_init.tstep;

if isempty(elapsed_time)
    elapsed_time = 0;
end


if(constraints_vec(2)>0)
    k_impact = 0; %%impact already happened
    elapsed_time = 0;
else
    if state == 14
        k_impact = max(floor((step_time - elapsed_time)/dT),1); %basically the 1 avoids requiring wrench on the right foot for a wrong preview on the step time
        if (elapsed_time + dT) <= step_time
            elapsed_time = elapsed_time + dT;
        end
    else
        k_impact = nsteps +1;
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

Pl   = w_H_l_sole(1:3,4);
w_R_l_sole     = w_H_l_sole(1:3,1:3);

Pr   = w_H_r_sole(1:3,4);
w_R_r_sole      = w_H_r_sole(1:3,1:3);



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

if state == 16
    COMdesfilt_pos = COMdesStateMachine(1:3,1);
    gains.COM = (mpc_init.gains.COM + mpc_init.gains.TerCOM);
    gains.COM(1:2,1) = gains.COM(1:2,1)*10;
    gains.COM(1:3,2) = gains.COM(1:3,2)*30;
else
    COMdesfilt_pos = [(Pl(1:2) + Pr(1:2))/2;COMdesStateMachine(3,1)]+ mpc_init.COMoffset;
end
 
%COMdesfilt = [COMdesfilt_pos,zeros(3,1)];

%if isempty(COMref_prev) || (constraints_vec(2)==0)
    COMref_prev = COMdesfilt_pos; %The desired position for the center of mass is updated only before the step
%end

f= zeros(12,1);
exitflag = 1;
COM_des_out = zeros(9,1);
COMref = zeros(3,1);
COM_last = zeros(9,1);
coder.extrinsic('solve_mpc_cvx')
[f(:),COM_des_out(:,:),exitflag(:),COMref(:,:),~,COM_last(:,:)] = solve_mpc_cvx(m, Cl, Bl, Cr, Br, ch_points, Pl, Pr, omega, 9.81, f_prev, zeros(3,1) , zeros(3,1), COMref_prev, mpc_init.COMoffset(1:2),minZ, gains, gamma0, nsteps, dT, k_impact); 

MPC_STEP = 0;
errorLastCom = norm(COMdesStateMachine(1:2,1) - COM_last(1:2)) + 2*norm(COM_last(4:5));
if  mpc_init.DECIDE_STEP == 1
    if ((errorLastCom > 0.08) || (exitflag <= 0))
        MPC_STEP = 1;
    end
    
end
