ROBOT_DOF = 23;

PORTS.IMU   = '/icub/inertial';


WBT_wbiList = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP';

sat.torque = 6;


ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
ROBOT_DOF        = 23;
addpath(genpath('../matlab'));

model.robot.dofs = ROBOT_DOF;
    
seesaw           = struct;
% Height of the seesaw
%  ____________
%  *          *  |
%   *        *   | h
%     *    *     |
%       **       |
seesaw.h         = 0.1;

%Radius of the seesaw
seesaw.rho       = 0.362;
% Distance beteewn the center of rotation and the center of mass
seesaw.delta     = seesaw.rho - seesaw.h + 0.002;
seesaw.inertia   = diag([7.6698599e-02, 3.7876787e-02, 1.0893139e-01]);
seesaw.mass      = 4.2;

seesaw.top       = 0.002;% seesaw.delta - (seesaw.rho - seesaw.h) ;

seesaw.kind      = seesawKind;
seesaw.lFootDistanceCenter      =  0.07;
seesaw.rFootDistanceCenter      = -0.07;

seesaw.lFootDistanceCenter      =  0.1025;
seesaw.rFootDistanceCenter      = -0.1025;

switch seesaw.kind
    case 1 %Spherical seesaw
        seesaw.iota      = seesaw.mass*inv(seesaw.inertia);
        seesaw.invIota   = inv(seesaw.iota);
    case 2 %Semi cylidrical seesaw
        seesaw.iota      = [1;0;0]*[1;0;0]'*seesaw.mass*inv(seesaw.inertia);
        seesaw.invIota   = 0;
end

reg             = struct;
reg.pinvTol     = 1e-7;

reg.pinvDamp    = 1e-2;
reg.pinvDampA   = 1e-7;
reg.pinvDampA   = 1e-4;
reg.HessianQP   = 1e-5;
model.seesaw    = seesaw;
%    
% INERTIA TENSOR:
% Ixx Ixy Ixz 7.6698599e-02 0.0000000e+00 0.0000000e+00
% Iyx Iyy Iyz 0.0000000e+00 3.7876787e-02 0.0000000e+00
% Izx Izy Izz 0.0000000e+00 0.0000000e+00 1.0893139e-01

%%Assumptions:
% 1) feet do not move with respect to the seesaw
% 2) feet are centered with the center of mass of the seesaw
% these assumptions are enforced (?) in the function as persistent
% variables

directionOfOscillation            = [0;1;0];
referenceParams                   = [0.00  0.1];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz

noOscillationTime        = 0; % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                               % that the robot waits before starting the left-and-righ

%%Gains and references
gain.posturalProp  = diag([ 10   10   20, 10   10    10    8, 10   10    10    8, 30   30   20    20    0 0, 30   50   30    60    0 0]);
                        
gain.posturalDamp  = gain.posturalProp*0;

gain.PAngularMomentum  = 0 ;
gain.DAngularMomentum  = 1;

gain.PCOM                 = diag([  10    25   20]);
gain.DCOM                 = 2*sqrt(gain.PCOM)/10;
gain.ICOM                 = diag([  0    0   0]);


gain.seesawKP          = 0.1;
gain.seesawKD          = 2*sqrt(gain.seesawKP);


reg.impedances             = 0.1;
reg.dampings               = 0;

% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

%physical size of foot
                             
gain.footSize                = [ -0.07 0.07   ;   % xMin, xMax
                                 -0.03 0.03 ];  % yMin, yMax    

fZmin                        = 20;


