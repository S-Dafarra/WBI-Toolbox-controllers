function [LfootPos_error,LfootOri_error,RfootPos_error,RfootOri_error,CoMPos_error, cartesian_err] = fromFwdKinToPosAndOri...
         (fwdkin_CoM,fwdkin_lfoot,fwdkin_lfootInit,fwdkin_rfoot,fwdkin_rfootInit,lsole_H_rsole,rsole_H_lsole)
%% Position errors
lFootPos         = fwdkin_lfoot(1:3,4);
rFootPos         = fwdkin_rfoot(1:3,4);
lFootPos_init    = fwdkin_lfootInit(1:3,4);
rFootPos_init    = fwdkin_rfootInit(1:3,4);

CoMPos_error     = fwdkin_CoM(1:3,4);
LfootPos_error   = lFootPos-lFootPos_init;
RfootPos_error   = rFootPos-rFootPos_init;

%% Rotation matrices at feet
R_lfoot          = fwdkin_lfoot(1:3,1:3);
R_rfoot          = fwdkin_rfoot(1:3,1:3);
R_lfoot_init     = fwdkin_lfootInit(1:3,1:3);
R_rfoot_init     = fwdkin_rfootInit(1:3,1:3);

% feet orientation is parametrized with Euler angles
[~,lFootOri]          = parametrization(R_lfoot);
[~,rFootOri]          = parametrization(R_rfoot);
[~,rFootOri_init]     = parametrization(R_rfoot_init);
[~,lFootOri_init]     = parametrization(R_lfoot_init);

%% Feet orientation errors
LfootOri_error        = lFootOri-lFootOri_init;
RfootOri_error        = rFootOri-rFootOri_init;

%% Foot Cartesian Position Error
[~,lFootOri_des]     = parametrization(R_rfoot*rsole_H_lsole(1:3,1:3));   %parametrization with Euler angles of the left foot  desired orientations
[~,rFootOri_des]     = parametrization(R_lfoot*lsole_H_rsole(1:3,1:3));   %parametrization with Euler angles of the right foot  desired orientations
cartesian_err      = zeros(3,4);
cartesian_err(:,1) = lFootPos - (rFootPos + rsole_H_lsole(1:3,4)); %cartesian error for the left foot
cartesian_err(:,2) = (lFootOri - lFootOri_des)';
cartesian_err(:,3) = rFootPos - (lFootPos + lsole_H_rsole(1:3,4)); %cartesian error for the right foot
cartesian_err(:,4) = (rFootOri - rFootOri_des)';

end