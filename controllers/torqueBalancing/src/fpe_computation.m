function [theta_bar, x, fpe_poly, fpe_poly_offset, dissipation] = fpe_computation(m, COMx, COM_vel, J_COM, g, foot, offset)
 %[theta_bar, x, fpe_poly, ctheta, test] = fpe_computation(m, COMx, COM_vel, J_COM, g, foot);
 % This function computes the residual of the foot placement estimator
% formula
%persistent in_guess;

theta_bar = 0;



% if isempty(in_guess)
%     in_guess = theta_bar;
% end


z = [0;0;1];        % vertical axis versor

n = cross(COM_vel,z);

n_norm = norm(n,2);
if (n_norm>0)
     n = n/n_norm;
else n = zeros(3,1);
end

h_dot = COM_vel(3); % vertical speed of the COM, COM_vel'*h

h = COMx(3);

J = n'*J_COM*n;     % COM rotational inertia projected on the plane
%theta_dot = COM_vel(4:6)'*n; % COM rotational speed on the plane


x = cross(z,n);     % horizontal vector on the plane

vx = COM_vel(1:3)'*x;

V = norm([vx,h_dot],2);

COM_proj = zeros(2,1);
foot_proj = zeros(2,1);

COM_proj(1) = COMx'*x;
COM_proj(2) = COMx'*z;

foot_proj(1) = foot'*x;
foot_proj(2) = foot'*z;

theta_dot = vx/h; %V/R, R=V/Vx*h
%theta_dot = (vx*(h)-h_dot*(COM_proj(1)-foot_proj(1)))/(h^2+(COM_proj(1)-foot_proj(1))^2);
%theta_dot = V/(h^2+(COM_proj(1)-foot_proj(1))^2);

% coder.extrinsic('solve_fpe')
% theta_bar = double(solve_fpe(theta_dot, m, h, vx, h_dot, J, g, in_guess));
% in_guess = theta_bar;

p = [(J^2*theta_dot^2 + 2*m*g*h*J); (2*m*h*V*J*theta_dot -2*m*g*h*J); (2*m^2*g*h^3 + m^2*h^2*V^2); -2*m^2*g*h^3]';

ctheta_in = roots(p);

ctheta = complex(zeros(3,1),zeros(3,1));
for k=1:min(3,length(ctheta_in))
    ctheta(k) = ctheta_in(k);
end
fpe_poly = zeros(3,1);
fpe_poly_temp = zeros(3,1);
fpe_max = 0;
ctheta_fin = 0;

for k = 1:length(ctheta)
        if ((abs(imag( ctheta(k) ))<0.001) && (ctheta(k)> 0) && (ctheta(k) <= 1))
            
            theta_bar = acos(real(ctheta(k)));
            fpe_poly_temp = h*tan(theta_bar)*x + [COMx(1);COMx(2);0];
            
            if (norm(fpe_poly_temp,2) > fpe_max)
                
                fpe_poly = fpe_poly_temp;
                fpe_max = norm(fpe_poly,2);
                ctheta_fin = real(ctheta(k));
                
            end
        end
end

fpe_poly_offset = h*tan(theta_bar + offset)*x + [COMx(1);COMx(2);0];

l = h/ctheta_fin;
dissipation = (m*l^2*(2*ctheta_fin^2 -1)+J)/(m*l^2+J);

end
