function[dataStore] = pathPlanner(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
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
                   'measuredPose',[],...
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
global isLab;

isLab = 0;

if isLab == 1
    maxV = 0.2;
    map = load('ExampleLabMap_2014.mat');
else
    maxV = 0.5;
    map = load('ExampleMap2_2014.mat');
end
robotRad = 0.16;

% map = load('ExampleMap2_2014.mat');
buffer = robotRad;
% [px, py, pt] = OverheadLocalizationCreate(tagNum);
% currPose = [px; py; pt];

walls = map.map;
optWalls = map.optWalls;
%
% optWalls = optWalls(2,:);
% optWalls = [optWalls(2,:); optWalls(6,:)];

% optWalls = [];
wpOrWall = 1;

regWps = map.waypoints;
regWps = regWps.';
ecWps = map.ECwaypoints;
ecWps = ecWps.';
wps = [regWps ecWps];

% wps = [-.5; -3.5];

unvisitedWPs = wps;
visitedWPs = [];
wpOrWall = 1;

% [path pathDist relocalize] = planPath(walls,optWalls,wpOrWall,currPose,unvisitedWPs,visitedWPs );

flag = 0;
% if isnan(path)
%     flag = 1;
% else
%     waypoints = path(2:end,2:3);
% end

% waypoints = [-1 0; 1 0];
% m = size(waypoints, 1);
closeEnough = 0.2;
gotopt = 1;
stop = 0;
dontMove = 0;


tic
while toc < maxTime
    if (flag == 1)
        break
    end
    if (isempty(unvisitedWPs) && isempty(optWalls))
        break
    end
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    %% Ben's Code
    %
    %
    %
    %%
        ptime = dataStore.truthPose(end,1);
        px = dataStore.truthPose(end,2) + (rand*.5 - .25); % (rand - .5);
        py = dataStore.truthPose(end,3) + (rand*.5 - .25); %(rand - .5);
        pt = dataStore.truthPose(end,4) + (rand*.5 - .25); %(rand - .5);
        currLoc = [px; py; pt];
        
    if (wpOrWall == 1) || (wpOrWall == 2)
%         px = dataStore.truthPose(end,2) + (rand - .5);
%         py = dataStore.truthPose(end,3) + (rand - .5);
%         pt = dataStore.truthPose(end,4) + (rand - .5);
%         currLoc = [px; py; pt];
        [path pathDist relocalize removeIdx wpOrWall] = planPath(walls,optWalls,wpOrWall,currLoc,unvisitedWPs,visitedWPs );
        if (relocalize == 1)
            continue
        end
        if (isnan(path))
            continue
        end
        if (wpOrWall == 1)
            wpOrWall = 3;
        elseif (wpOrWall == 2)
            wpOrWall = 4;
        end
        waypoints = path(2:end,2:3);
        m = size(waypoints, 1);
        gotopt = 1;
        dontMove = 0;
    end
    %%
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        if stop == 0
            currPose = [ptime px py pt];
            dataStore.measuredPose = [dataStore.measuredPose; currPose];
            
            if (wpOrWall == 4) && ((gotopt == m) || (gotopt == (m - 1))) && ((dataStore.bump(end,2) == 1) || (dataStore.bump(end,3) == 1) || (dataStore.bump(end,7) == 1))
                SetFwdVelAngVelCreate(CreatePort, 0,0 );
                dontMove = 1;
                walls = [walls; optWalls(removeIdx,:)];
                optWalls(removeIdx,:) = [];
                wpOrWall = 1;
%             end
             elseif ((dataStore.bump(end,2) == 1) || (dataStore.bump(end,3) == 1) || (dataStore.bump(end,7) == 1))
                %% here we handle the case when the closest point goes through an optional wall
                if (gotopt == 1)
                elseif (size(optWalls,1) >= 1)
                    isectIdx = 0;
                    for k = 1:size(optWalls,1)
                        ax1 = waypoints(gotopt,1);
                        ax2 = waypoints((gotopt - 1),1);
                        ay1 = waypoints(gotopt,2);
                        ay2 = waypoints((gotopt - 1),2);
                        wx1 = optWalls(k,1);
                        wy1 = optWalls(k,2);
                        wx2 = optWalls(k,3);
                        wy2 = optWalls(k,4);
                        [isect x y ua] = intersectPoint(ax1,ay1,ax2,ay2,wx1,wy1,wx2,wy2);
                        if (isect == true)
                            isectIdx = k;
                            break
                        end
                    end
                    if (isectIdx ~= 0)
                        SetFwdVelAngVelCreate(CreatePort, 0,0 );
                        dontMove = 1;
                        walls = [walls; optWalls(isectIdx,:)];
                        optWalls(isectIdx,:) = [];
                        wpOrWall = 1;
                    end 
                end
                %%
            elseif (abs(currPose(end,2) - waypoints(gotopt,1)) < closeEnough ) && (abs(currPose(end,3) - waypoints(gotopt,2)) < closeEnough)
                if gotopt < m
                    gotopt = gotopt + 1;
                else
                    %%
                    SetFwdVelAngVelCreate(CreatePort, 0,0 );
                    if (wpOrWall == 3)
                        visitedWPs = [visitedWPs unvisitedWPs(:,removeIdx)];
                        unvisitedWPs(:,removeIdx) = [];
                        wpOrWall = 1;
                    elseif (wpOrWall == 4) && ((dataStore.bump(end,2) == 1) || (dataStore.bump(end,3) == 1) || (dataStore.bump(end,7) == 1))
                        walls = [walls; optWalls(removeIdx,:)];
                        optWalls(removeIdx,:) = []; 
                        wpOrWall = 1;
                    elseif (wpOrWall == 4)
                        optWalls(removeIdx,:) = []; 
                        wpOrWall = 1;
                    end
%                     figure(3)
%                     hold on
%                     plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'g-');
%                     plot(dataStore.measuredPose(:,2),dataStore.measuredPose(:,3),'r-');
%                     hold off
                    dontMove = 1;
                    %return
                end
            end
            if (dontMove == 0 )
            [velX, velW] = updateVW(waypoints, gotopt, currPose);
%             turnAngle(CreatePort, .2, velW);
            [cmdV,cmdW] = limitCmds(velX,velW,maxV,robotRad);
            if abs(cmdW) > .4
                cmdWNew = .4*(cmdW/abs(cmdW));
                cmdVNew = cmdV*(cmdWNew/cmdW);
                cmdW = cmdWNew;
                cmdV = cmdVNew;
            end
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
%             SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
            end
        end
    end
    pause(.1);
%     SetFwdVelAngVelCreate(CreatePort, 0,0);
    pause(.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
