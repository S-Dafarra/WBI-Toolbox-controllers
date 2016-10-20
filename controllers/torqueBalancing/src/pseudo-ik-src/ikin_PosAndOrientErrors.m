function [iKinFeetCorr,iKinComCorr,iKinSoleCorr,iKinPostCorr]  = ikin_PosAndOrientErrors(lFootPos,rFootPos,rFootOri,lFootOri, cartesian_err, xCoM,qj, CoMRefsDes,posturesDes,...
                                                                            lFootVel,rFootVel,dxCoM,dqj,gain)
%% Feet errors
iKinFeetCorr    = [(-gain.ikin.kpfeet*lFootPos(1:3)-gain.ikin.kdfeet*lFootVel(1:3));
                   (-gain.ikin.kpfeet*lFootOri(1:3)'-gain.ikin.kdfeet*lFootVel(4:6));
                   (-gain.ikin.kpfeet*rFootPos(1:3)-gain.ikin.kdfeet*rFootVel(1:3));
                   (-gain.ikin.kpfeet*rFootOri(1:3)'-gain.ikin.kdfeet*rFootVel(4:6))];
               
%% CoM errors                                         
iKinComCorr     = CoMRefsDes(:,3)-gain.ikin.kp*(xCoM-CoMRefsDes(:,1))-gain.ikin.kd*(dxCoM-CoMRefsDes(:,2));

%% Cartesian error for the swing foot

iKinSoleCorr    = [(-gain.ikin.kpsole*cartesian_err(:,1) -gain.ikin.kdsole*lFootVel(1:3));
                   (-gain.ikin.kpsole*cartesian_err(:,2) -gain.ikin.kdsole*lFootVel(4:6));
                   (-gain.ikin.kpsole*cartesian_err(:,3) -gain.ikin.kdsole*rFootVel(1:3));
                   (-gain.ikin.kpsole*cartesian_err(:,4) -gain.ikin.kdsole*rFootVel(4:6))];
               

%% Posture errors
iKinPostCorr    = posturesDes(:,3)-gain.ikin.impedances*(qj-posturesDes(:,1))-gain.ikin.dampings*(dqj-posturesDes(:,2));
end