function [x_sat, SAT]=step_saturation(x,max_d)
%%% This function takes as input a 2-D vector corresponding to the position
%%% of a point on the plane. If the distance from the origin is greater
%%% than max_d, it outputs the saturated version of x which mantains the
%%% same ratio between x(1) and x(2) and at a max_d distance from the
%%% origin.
x_sat = zeros(2,1);
ld=norm(x(1:2),2);

if ld > max_d
    x_sat = max_d/ld * x;
    SAT = 1;
else x_sat = x;
    SAT = 0;
end
end
