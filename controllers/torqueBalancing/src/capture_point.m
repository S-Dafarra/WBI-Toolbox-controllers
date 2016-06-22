function r_CP = capture_point(STEP, r_ic, r_CxP, omega, robot_step_time, delta_t)
 persistent r_ic0;
 persistent elapsed_time;
 
 if isempty(r_ic0)
     r_ic0 = r_ic;
     elapsed_time = 0;
 end
 
 if (STEP == 0)
     r_ic0 = r_ic;
     r_CP = r_ic;
     elapsed_time = 0;
 
 else  r_CP = (r_ic0 - r_CxP)*exp(omega * (robot_step_time-elapsed_time)) + r_CxP;
       elapsed_time = elapsed_time + delta_t;
 end
end
     
     