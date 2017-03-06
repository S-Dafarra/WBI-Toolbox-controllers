x0 = [0,2,3]';
v0 = zeros(3,1);
xf = ones(3,1);
vf = v0;
af = v0;
dT = 0.01;
nsteps =100;
k_impact = 101;
tic
[x_des,v_des] = fourth_traj(x0,v0,xf,vf,af,dT,1,nsteps);
toc
time = dT:dT:dT*nsteps;
figure
plot(time,x_des(1:3,1:nsteps))
figure
plot(time,v_des(1:3,1:nsteps))