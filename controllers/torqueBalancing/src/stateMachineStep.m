function [w_H_b, CoMDes,qDes,constraints,impedances,kpCom,kdCom,...
    currentState,jointsSmoothingTime, QP_OFF, SMOOTH, SMOOTH_com] = ...
    stateMachineStep(CoM_0, q0, l_sole_CoM,r_sole_CoM,qj, t, ...
                  wrench_rightFoot,wrench_leftFoot,l_sole_H_b, r_sole_H_b,...
                  sm,gain, STEP_DOWN, r_CxP, COM_l_v, MAKE_A_STEP,...
                  q_step_legs, COM_ref, l_solex, r_solex)
    %#codegen
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;
    persistent t_previous;
    persistent COM_prev_l;
    persistent t_debounce;
    persistent COMconstRef;
    
    if isempty(state) || isempty(tSwitch) || isempty(w_H_fixedLink) 
        state         = sm.stateAt0;
        tSwitch       = 0;
        w_H_fixedLink = eye(4);
        COM_prev_l = CoM_0;
        t_previous = -1;
        t_debounce = -1;
        COMconstRef = -ones(3,1);
    end
    
    CoMDes      = CoM_0;
    constraints = [1; 1];    
    qDes        = q0;
    w_H_b       = eye(4);
    impedances  = gain.impedances(1,:);
    kpCom       = gain.PCOM(1,:);   
    kdCom       = gain.DCOM(1,:);  
    
    QP_OFF = 0;
    SMOOTH = 1;
    SMOOTH_com = 1;
    q_before = zeros(12,1);

    %% TWO FEET BALANCING
    if state == 1 
        w_H_b      =  w_H_fixedLink * l_sole_H_b;

        if t > sm.tBalancing %after tBalancing time start moving weight to the left
           state = 2;
           if sm.demoOnlyRightFoot
                w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
                state = 8;
           end
           
        end
    end

    %% TRANSITION TO THE LEFT FOOT
    if state == 2 
        w_H_b      =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes     = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        
        impedances = gain.impedances(state,:);
        kpCom      = gain.PCOM(state,:);   
        kdCom      = gain.DCOM(state,:);   

        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        
        CoMError   = fixed_link_CoMDes(1:3) - l_sole_CoM(1:3);
        
        qDes       = sm.joints.states(state,:)'; % new reference for q
        
        if norm(CoMError(2)) < sm.com.threshold && wrench_rightFoot(3) < sm.wrench.thresholdContactOff
           state = 3; 
           tSwitch = t;
        end
    end

    %% LEFT FOOT BALANCING 
    if state == 3 
        w_H_b      =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes     = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
 
        
        constraints = [1; 0]; %right foot is no longer a constraints

        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        if t > tSwitch + sm.DT % yoga
            state   = 4;
            tSwitch = t;
            if sm.skipYoga
                state   = 5;
            end
        end
    end
    
    %% YOGA LEFT FOOT
    if state == 4 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;


        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        
        constraints = [1; 0]; %right foot is no longer a constraints

        qDes        =  sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        if t >  tSwitch
                qDes = sm.joints.pointsL(1,2:end)';
        end
          
        if STEP_DOWN~=0
           state   = 5;
           tSwitch = t;
        end
        
        if MAKE_A_STEP
            state = 14;
            tSwitch = t;
        end       
        
        
    end
    
    %% PREPARING FOR SWITCHING
    if state == 5 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        
        constraints = [1; 0]; %right foot is no longer a constraints

        qDes        =  sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        qTildeRLeg  = qj(end-5:end)-qDes(end-5:end);
        
        qTildeLLeg  = qj(end-11:end-6)-qDes(end-11:end-6);
            
        if norm(qTildeRLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTildeLLeg)*180/pi < sm.joints.thresholdInContact
            state   = 6;
            tSwitch = t;
        end
    end
    
    %% LOOKING FOR A CONTACT
    if state == 6 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        
        constraints = [1; 0]; %right foot is no longer a constraints

        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        if wrench_rightFoot(3) > sm.wrench.thresholdContactOn
            state = 7;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 7 
        w_H_b       =  w_H_fixedLink * l_sole_H_b;
        w_H_b_r     =  w_H_b/r_sole_H_b;        
        
        %CoMDes      = 0.5*([w_H_fixedLink(1:2,4);CoM_0(3)] + [w_H_b_r(1:2,4);CoM_0(3)]); %+ sm.com.states(state,:)';         
        CoMDes      = 0.5*([l_solex(1:2);CoM_0(3)] + [r_solex(1:2);CoM_0(3)]);
        
        constraints = [1; 1]; 
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:); 
                
        if STEP_DOWN == 0
           state   = 2;
           tSwitch = t;
        end
    end
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
     
%% TRANSITION TO THE RIGHT FOOT
    if state == 8 
        constraints = [1; 1]; %right foot is no longer a constraints
        w_H_b       =  w_H_fixedLink*r_sole_H_b;


        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
 
        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        
        CoMError    = fixed_link_CoMDes(1:3) - r_sole_CoM(1:3);
        
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   
        qDes        =  sm.joints.states(state,:)';

        if norm(CoMError(2)) < sm.com.threshold  && wrench_leftFoot(3) < sm.wrench.thresholdContactOff
           state = 9; 
           tSwitch = t;
        end

    end
    
     %% RIGHT FOOT BALANCING 
    if state == 9
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b       =  w_H_fixedLink*r_sole_H_b;

        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   
        if t > tSwitch + sm.DT % yoga
            state   = 10;
            tSwitch = t;
            if sm.skipYoga
                state   = 11;
            end
        end
    end
    
    %% YOGA RIGHT FOOT
    if state == 10 
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b   =  w_H_fixedLink*r_sole_H_b;

        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        =  sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        for i = 1: size(sm.joints.pointsR,1)-1
            if t > (sm.joints.pointsR(i,1) + tSwitch) && t <= (sm.joints.pointsR(i+1,1)+ tSwitch)
                qDes = sm.joints.pointsR(i,2:end)';
            end
        end
        if t > sm.joints.pointsR(end,1) + tSwitch 
            qDes = sm.joints.pointsR(end,2:end)';
            if  (t > sm.joints.pointsR(end,1) + tSwitch + sm.jointsSmoothingTimes(state) + sm.joints.pauseTimeLastPostureR ) 
                state   = 11;
                tSwitch = t;
            end
        end
    end
    
    %% PREPARING FOR SWITCHING
    if state == 11 
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b       =  w_H_fixedLink*r_sole_H_b;

        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        =  sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        qTildeRLeg  = qj(end-5:end)-qDes(end-5:end);
        
        qTildeLLeg  = qj(end-11:end-6)-qDes(end-11:end-6);
            
        if norm(qTildeRLeg)*180/pi < sm.joints.thresholdInContact && norm(qTildeLLeg)*180/pi < sm.joints.thresholdNotInContact
            state   = 12;
            tSwitch = t;
        end
    end
    
    %% LOOKING FOR A CONTACT
    if state == 12
        constraints = [0; 1]; %left foot is no longer a constraints
        w_H_b       =  w_H_fixedLink*r_sole_H_b;

        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        = sm.joints.states(state,:)';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   

        if wrench_leftFoot(3) > sm.wrench.thresholdContactOn 
            state   = 13;
            tSwitch = t;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 13
        w_H_b       =  w_H_fixedLink*r_sole_H_b;
        constraints = [1; 1]; %right foot is no longer a constraints
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:);   
        if t - tSwitch> sm.tBalancing %after tBalancing time start moving weight to the left
           if sm.yogaInLoop
              state = 2; 
              w_H_fixedLink   = w_H_fixedLink*r_sole_H_b/l_sole_H_b;
              if sm.demoOnlyRightFoot
                 state = 8;           
                 w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
              end
           end
        end
    end 
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% FALLING STATE (step with the right foot)
    if state == 14
    
    QP_OFF = 0;
    SMOOTH = 1;
    SMOOTH_com = 0;
    
    w_H_b       =  w_H_fixedLink * l_sole_H_b;
    constraints = [1; 0];
    
    impedances  = gain.impedances(state,:);
    kpCom       = gain.PCOM(state,:);   
    kdCom       = gain.DCOM(state,:);
    qDes        = [sm.joints.states(state,1:11),q_step_legs']';
    %qDes = sm.joints.pointsL(1,2:end)';
    
    if t_previous < 0
        t_previous = t;
    end
    
    sim_pend = COM_prev_l(1:2) + (t-t_previous)*COM_l_v(1:2) + 0.5* (t-t_previous)^2 * 9.81/CoM_0(3) * (COM_prev_l(1:2) - r_CxP(1:2)); 
    %CoMDes      = [sim_pend;0*CoM_0(3)];
    CoMDes = [COM_ref(1:3)];
    t_previous = t;
    
    if wrench_rightFoot(3) < (sm.wrench.thresholdContactOn + 20)
        t_debounce = t;
    end
    if (t-t_debounce) > 0.02
            state = 15;
            tSwitch = t;
            t_previous = -1; %resetting
            q_before = q_step_legs;
            t_debounce = -1;
            COMconstRef = -ones(3,1);
           % w_H_fixedLink   = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
    end
    
    
    end
    
    %% RESTORING
    if state == 15 
        %w_H_b       =  w_H_fixedLink*r_sole_H_b;
        w_H_b       =  w_H_fixedLink * l_sole_H_b;
        w_H_b_r     =  w_H_b/r_sole_H_b;        
               
        %CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] - sm.com.states(15,:)';
        %CoMDes = [w_H_b_r(1:2,4);CoM_0(3)] + sm.com.states(15,:)';
        %CoMDes      = 0.5*([w_H_fixedLink(1:2,4);CoM_0(3)] + [w_H_b_r(1:2,4);CoM_0(3)]); %+ sm.com.states(state,:)';
        
        if COMconstRef(3) == -1
            COMconstRef = 0.5*([l_solex(1:2);CoM_0(3)] + [r_solex(1:2);CoM_0(3)]);            
        end
        
        CoMDes = COMconstRef;
            
        
%     if wrench_leftFoot(3) > (sm.wrench.thresholdContactOn)      
           constraints = [1; 1];

%          %SMOOTH_com = 1;
%          
%     else constraints = [0; 1];
%          %CoMDes = [w_H_b_r(1:2,4);CoM_0(3)];
%          %SMOOTH_com = 0;
%     end
    
        qDes        = [sm.joints.states(state,1:11),q_before']';
        impedances  = gain.impedances(state,:);
        kpCom       = gain.PCOM(state,:);   
        kdCom       = gain.DCOM(state,:); 
        
        SMOOTH = 1;        
        QP_OFF = 0;
        SMOOTH_com = 0; 
        
    end
    
     %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    COM_prev_l = l_sole_CoM;
    currentState        = state;
    jointsSmoothingTime = sm.jointsSmoothingTimes(state);
    