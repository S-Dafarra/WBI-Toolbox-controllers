function [fpe, fpe_alternative, theta_bar] = fpe_computation(m, COMx, COM_vel, J_COM, n, g, COPx)
% This function computes the residual of the foot placement estimator
% formula

theta_bar = 0;

n_norm = norm(n,2);
if (n_norm>0)
    n = n/n_norm;
else n = zeros(3,1);
end

h_dot = COM_vel(3); % vertical speed of the COM
h = COMx(3);

J = n'*J_COM*n;     % COM rotational inertia projected on the plane
%theta_dot = COM_vel(4:6)'*n; % COM rotational speed on the plane

z = [0;0;1];        % vertical axis versor

x = cross(z,n);     % horizontal vector on the plane

vx = COM_vel(1:3)'*x;

COM_proj = zeros(2,1);
COP_proj = zeros(2,1);

COM_proj(1) = COMx'*x;
COM_proj(2) = COMx'*z;

COP_proj(1) = COPx'*x;
COP_proj(2) = COPx'*z;

l = norm(COM_proj-COP_proj,2);
theta_dot = vx*(COM_proj(2)-COP_proj(2))/(l^2);

coder.extrinsic('solve_fpe')
theta_bar = double(solve_fpe(theta_dot, m, h, vx, h_dot, J, g));

fpe =h*tan(theta_bar)*x + [COMx(1);COMx(2);0];
fpe_alternative = [COPx(1);COPx(2);0] + 2*h*tan(theta_bar)*x;
end