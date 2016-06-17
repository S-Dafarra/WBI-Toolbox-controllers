function fpe = fpe_computation(m, COMx, COM_vel, J_COM, n, g)
% This function computes the residual of the foot placement estimator
% formula

theta_bar = 0;

n_norm = norm(n,2);
if (n_norm>0)
    n = n/n_norm;
else n = 0;
end

h_dot = COM_vel(3); % vertical speed of the COM
h = COMx(3);

J = n'*J_COM*n;     % COM rotational inertia projected on the plane
theta_dot = COM_vel(4:6)'*n; % COM rotational speed on the plane

z = [0;0;1];        % vertical axis versor

x = cross(z,n);     % horizontal vector on the plane

vx = COM_vel(1:3)'*x;

coder.extrinsic('solve_fpe')
theta_bar = double(solve_fpe(theta_dot, m, h, vx, h_dot, J, g));

fpe =h*tan(theta_bar)*x + [COMx(1);COMx(2);0]; 
end