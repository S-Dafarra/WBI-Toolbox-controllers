function [Mbb_COM] = COM_inertia_matrix(Mbb, base_origin, COMx)
% COM_inertia_matrix computes the inertia matrix of the robot computed
% about the COM, assumed to have the same orientation of the base frame

p = COMx - base_origin;

p_hat = [       0, -p(3),  p(2);
          p(3),        0, -p(1);
         -p(2),  p(1),        0];

Aj =  [     eye(3),  p_hat;
        zeros(3,3), eye(3)];

Mbb_COM = Aj'*Mbb*Aj;
end