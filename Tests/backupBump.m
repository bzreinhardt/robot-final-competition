function[dataStore] = backupBump(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
% BACKUPBUMP: commands robot to drive forward at constant velocity until it
% bumps into something. If a bump sensor is triggered, command the robot to
% back up 0.25 m and turn clockwise 30 degrees and then continue
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
 maxDuration= 1200;  % Max time to allow the program to run (s)
 maxDistSansBump= 5; % Max distance to travel without obstacles (m)
maxV = 0.3;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
    maxVelIncr= 0.005;  % Max incrementation of forward velocity (m/s)
    maxOdomAng= pi/4;   % Max angle to move around a circle before 
                        % increasing the turning radius (rad)
     Wheel2center = 0.13;                   
                        
% Initiallize loop variables
tic
start = 'start'
distSansBump= 0;    % Distance traveled without hitting obstacles (m)
angTurned= 0;       % Angle turned since turning radius increase (rad)
v= 0;               % Forward velocity (m/s)


while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    
    % Check for and react to bump sensor readings
    bumped= bumpCheckReact(CreatePort,maxV);
    

    
    % move forward if not bumped 
    
    cmdV = 0.25;
    cmdW = 0;
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV,Wheel2center);
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

end

function bumped= bumpCheckReact(serPort,maxV)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster, ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    bumped= BumpRight || BumpLeft || BumpFront;
    
    dist = -0.25;
    ang = (-30);
    turnV = 0.1;
    % Halt forward motion and turn only if bumped
    if bumped
        %back up for 0.25m
        travelDist(serPort, maxV, dist);
% Turn away from obstacle
        turnAngle(serPort, turnV, ang);
    end
end
