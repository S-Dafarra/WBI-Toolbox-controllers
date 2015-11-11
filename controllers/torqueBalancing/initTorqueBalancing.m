% clear all;
clc;

robotName = 'icub';            
localName = 'matlabTorqueBalancing';

simulationTime    = inf;       % Simulation time in seconds

USE_QP_SOLVER    = 0;

LEFT_RIGHT_FOOT_IN_CONTACT  = [1 0];

DEMO_MOVEMENTS      = 1;  % Either 0 or 1 

SMOOTH_DES_COM      = 0;   % If equal to one, the desired streamed values of the center of mass are smoothed internally through mimum jerk trajectory
SMOOTH_DES_Q        = 1;   % If equal to one, the desired streamed values of the postural tasks are smoothed internally through mimum jerk trajectory
SMOOTH_CONSTRAINTS  = 1;   % If equal to one, the desired streamed values of the logic vector identifying the active constraints are smoothed internally through mimum jerk trajectory

USE_SM = true; % If equal to true, the (internal) state machine will be used. The robot will switch from 2 to 1 (left) foot
               % PLEASE, use logical values (true or false)
% Controller period
Ts                = 0.01; % [s]

% Load gains and parameters for the specific robot
run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
addpath('extra/')
[ConstraintsMatrix,bVectorConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);

if DEMO_MOVEMENTS == 1
    robotSpecificReferences = fullfile('robots',getenv('YARP_ROBOT_NAME'),'references.m');
    if exist(robotSpecificReferences, 'file')
        run(robotSpecificReferences);
    else
        %just try to launch the one in the path (?)
        references;
    end
end

if USE_SM == true
    initStateMachine;
end

% If you want to sync Gazebo and simulink, 
% remember that you have to launch gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
%
% Also, open the subsystem "Synchronizer" in the simulonk model 
% "balancingOptTau.mdl" and comment the block "Real Time Syncronizer" and
% uncomment the block "ySynchronizer".