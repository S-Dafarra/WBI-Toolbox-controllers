function [x_des,v_des] = fourth_traj(x0,v0,xf,vf,af,dT,k_impact,nsteps)

if(size(x0,1) < size(x0,2)) %all the vectors should be coloumn vectorss
   x0 = x0';
   v0 = v0';
   xf = xf';
   vf = vf';
   af = af';
end
T = dT*k_impact;
time = dT:dT:T;

k0 = x0;
k1 = v0;
k2 = af/2 - (6*(x0 - xf) + 3*T*(v0+vf))/T^2;
k3 = (-af*T^2+(3*v0+5*vf)*T+8*(x0-xf))/T^3;
k4 = -(-af/2*T^2 + (v0+2*vf)*T+3*(x0-xf))/T^4;

x_des = zeros(length(x0),max(k_impact,nsteps));
v_des = zeros(length(v0),max(k_impact,nsteps));

for i=1:length(time)
x_des(:,i) = k0 + k1*time(i) + k2*time(i)^2 + k3*time(i)^3 + k4*time(i)^4;
v_des(:,i) = k1 + 2*k2*time(i) + 3*k3*time(i)^2 + 4*k4*time(i)^3;
end

if(nsteps > k_impact)
    x_des(:,k_impact+1:end) = repmat(xf,1,nsteps-k_impact);
    v_des(:,k_impact+1:end) = repmat(vf,1,nsteps-k_impact);
end
    