function r_eq = equivalent_r (r, delta_t, omega, OUT)

persistent running_r;
persistent running_time;

if isempty(running_r) || (OUT == 0)
    running_r = zeros ( length(r),1);
    running_time = 0;    
end

w = (omega*exp(delta_t)-1)/(omega*exp(running_time + delta_t) -1);
r_eq = (1-w)*running_r + w*r;

running_r = r_eq;
running_time = running_time + delta_t;

end