function r_CP = capture_point(STEP, r_ic, r_CxP, omega, robot_step_time)
 persistent r_ic0;
 
 
 if isempty(r_ic0)
     r_ic0 = r_ic;
 end
 
 if (STEP == 0)
     r_ic0 = r_ic;
     r_CP = r_ic;    
 
 else  r_CP = (r_ic0 - r_CxP)*exp(omega * robot_step_time) + r_CxP;
       
 end
end
     
     