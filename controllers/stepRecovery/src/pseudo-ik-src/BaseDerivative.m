function BaseDer   = BaseDerivative(NuBase_Ref, baseQuat)
% apply the conversion of reference frame between base and wolrd
omegaBase      = NuBase_Ref(4:end);
[~,RBase]      = frame2posrot(baseQuat);

omegaWorld     = (RBase')*omegaBase; %this passage is needed because the derivative of the quaternion is based on the left trivialized angular velocity 

% calculate the quaternion derivative
quat_base      = baseQuat(4:end);

dquat_base     = quaternionDerivative(quat_base,omegaWorld,1);

% calculate the base derivative
BaseDer        = [NuBase_Ref(1:3);dquat_base];

end