function dotNu_ikin   = Ikin_Acceleration_generator(Jc, Jcom, Jsole, Jp, dJcNu, dJcomNu, dJsoleNu, iKinFeetCorr, iKinComCorr, iKinSoleCorr, iKinPostCorr)

%setup parameters
n_joints  = length(Jcom(1,7:end));
PINV_TOL  = 1e-5;  

% null space projectors for primary and secondary task
Nc    = eye(6+n_joints) - pinv(Jc,PINV_TOL)*Jc;
Ncom  = eye(6+n_joints) - pinv(Jcom*Nc,PINV_TOL)*Jcom*Nc;
Nsole = eye(6+n_joints) - pinv(Jsole*Ncom*Nc,PINV_TOL)*Jsole*Ncom*Nc;

%reference acceleration calculation
dotNu_feet   = pinv(Jc,PINV_TOL)*(iKinFeetCorr - dJcNu);

dotNu_sole   = pinv(Jsole*Nc)*(iKinSoleCorr - dJsoleNu - Jsole*dotNu_feet); 

dotNu_com    = pinv(Jcom*Nc*Nsole, PINV_TOL)*(iKinComCorr - dJcomNu - Jcom*dotNu_feet - Jcom*Nc*dotNu_sole);

dotNu_post   = pinv(Jp*Nc*Nsole*Ncom, PINV_TOL)*(iKinPostCorr - Jp*dotNu_feet -Jp*Nc*dotNu_sole-Jp*Nc*Nsole*dotNu_com);

dotNu_ikin   = dotNu_feet + Nc*(dotNu_sole + Nsole*(dotNu_com + Ncom*dotNu_post));

end

% %reference acceleration calculation
% dotNu_feet   = pinv(Jc,PINV_TOL)*(iKinFeetCorr - dJcNu);
% 
% dotNu_com    = pinv(Jcom*Nc, PINV_TOL)*(iKinComCorr - dJcomNu - Jcom*dotNu_feet);
% 
% dotNu_sole   = pinv(Jsole*Nc*Ncom)*(iKinSoleCorr - dJsoleNu - Jsole*dotNu_feet - Jsole*Nc*dotNu_com); 
% 
% dotNu_post   = pinv(Jp*Nc*Ncom*Nsole, PINV_TOL)*(iKinPostCorr - Jp*dotNu_feet -Jp*Nc*dotNu_com-Jp*Nc*Ncom*dotNu_sole);
% 
% dotNu_ikin   = dotNu_feet + Nc*(dotNu_com + Ncom*(dotNu_sole + Nsole*dotNu_post));