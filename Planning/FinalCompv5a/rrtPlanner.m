function[dataStore] = rrtPlanner(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
% VISTWAYPOINTS: Uses feedback and feedback linearization to steer the
% robot toward a list of waypoints.
% 
%   
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
maxV = 0.5;
robotRad = 0.5;

    map = load( 'cornerMap.mat');
    map = map.map;
    buffer = robotRad;
%     [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
%     start = [dataStore.truthPose(1,2) dataStore.truthPose(1,3)];
    [px, py, pt] = OverheadLocalizationCreate(tagNum);
    start = [px py];
    goal = [2 2.5];
    path = buildRRT(map,start,goal,buffer);
    flag = 0;
    if isnan(path)
        flag = 1;
    else
        waypoints = path(2:end,2:3);
    end

% waypoints = [-1 0; 1 0];
m = size(waypoints, 1);
closeEnough = 0.1;
gotopt = 1;
stop = 0;

tic
while toc < maxTime
    if (flag == 1)
        break
    end
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)

    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        if stop == 0
            currPose = dataStore.truthPose;
            if (abs(currPose(end,2) - waypoints(gotopt,1)) < closeEnough ) && (abs(currPose(end,3) - waypoints(gotopt,2)) < closeEnough)
                if gotopt < m
                    gotopt = gotopt + 1;
                else
                    SetFwdVelAngVelCreate(CreatePort, 0,0 );
                    hold on
                    plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'g-');
                    hold off
                    return
                end
            end
                [velX, velW] = updateVW(waypoints, gotopt, currPose);
                [cmdV,cmdW] = limitCmds(velX,velW,maxV,robotRad);
                SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
        end
    end
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
