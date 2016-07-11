function [x_sat, SAT]=step_saturation(x,max_d)
%%% This function takes as input a 2-D vector corresponding to the position
%%% of a point on the plane. If the distance from the origin is greater
%%% than max_d, it outputs the saturated version of x which mantains the
%%% same ratio between x(1) and x(2) and at a max_d distance from the
%%% origin.
ld=norm(x(1:2),2);
x_sat = zeros(3,1);
if ld > max_d
    x_sat(1:2) = max_d/ld * x(1:2);
    SAT = 1;
else x_sat(1:2) = x(1:2);
    SAT = 0;
end
x_sat(3) = x(3);
end
