function[dataStore] = visitWaypoints(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
% BUMPTURNNINTY: commands robot to drive forward at constant velocity until it
% bumps into something. If a bump sensor is triggered, command the robot to
% turn 90 degrees clockwise and continue
% 
%   dataStore = TURNINPLACE(CreatePort,SonarPort,BeaconPort,tagNum,maxTime) runs 
% 
%   INPUTS
%       CreatePort  Create port object (get from running RoombaInit)
%       SonarPort   Sonar port object (get from running RoombaInit)
%       BeaconPort  Camera port object (get from running RoombaInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    SonarPort = CreatePort;
    BeaconPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 3
    BeaconPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 5
    maxTime = 500;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'lidar', [], ...
                   'sonar', [], ...
                   'bump', [], ...
                   'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%set up constraints
maxV = 0.4;

epsilon = 0.2;
closeEnough = 0.1;
Wheel2center = 0.13;
% Initiallize loop variables
tic
start = 'start'




waypoints = [-1 0; 1 0];
%waypoints = [-3 0; 0 -3; 3 0; 0 3];

gotopt = 1;
while gotopt <= size(waypoints,1)
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    %find heading to move at desired velocity
    [px, py, pt] = OverheadLocalizationCreate(tagNum);
    v_x = waypoints(gotopt,1) - px;
    v_y = waypoints(gotopt,2) - py;
    
    [cmdV,cmdW] = feedbackLin(v_x, v_y, pt, epsilon);
  
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV,Wheel2center);
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    
    pause(0.1);
    [px, py, pt] = OverheadLocalizationCreate(tagNum);
    v_x = waypoints(gotopt,1) - px; v_y = waypoints(gotopt,2) - py;
    if (v_x^2+v_y^2)^.5 < (closeEnough + Wheel2center)
        gotopt = gotopt + 1;
    end
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

end