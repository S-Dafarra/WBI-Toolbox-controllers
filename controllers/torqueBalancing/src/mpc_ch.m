function K =mpc_ch(xch,ych)

K = zeros(length(xch),1);
K_out = convhull(xch,ych);
K(1:length(K_out)) = K_out;
