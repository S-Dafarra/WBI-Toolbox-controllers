function [OUT, xch, ych, K] = in_convex_hull(x,w_H_lsole, w_H_rsole, foot_size)

xm = foot_size(1,1);
xM = foot_size(1,2);
ym = foot_size(2,1);
yM = foot_size(2,2);

pl = w_H_lsole*[xM, xM, xm, xm;
                ym, yM, yM, ym;
                 0,  0,  0,  0;
                 1,  1,  1,  1];

pr = w_H_rsole*[xM, xM, xm, xm;
                ym, yM, yM, ym;
                 0,  0,  0,  0;
                 1,  1,  1,  1];
             
xch = [pl(1,:),pr(1,:)];
ych = [pl(2,:),pr(2,:)]; 

coder.extrinsic('in_ch')
OUT = 0;
K = zeros(8,1);
[OUT, K] = in_ch(x(1:2),xch,ych);
end
