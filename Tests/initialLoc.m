function[dataStore] = initialLoc(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
% initialLoc: localize robot to initial waypoint on map. 
% Details: Load the map. Initialize PF to the possible waypoints. Drive in
% a slow small circle until one of the points is clearly better than all of
% the other points. Set that as the robot's position.
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
                   'beacon', [],...
                   'ARs',[], 'sonars',[],'measurements',[],...
                   'predictMeas',[]);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%set up constraints
% maxTime = 500;  % Max time to allow the program to run (s)
maxV = 0.3;     % Max allowable forward velocity with no angular 
wheel2center = 0.13;  
sonarR = 0.16;
cameraR = 0;
slowV = 0;
%TODO load covariance matrix
load('ExampleMap1_2014.mat');
                        
% Initiallize loop variables
tic
start = 'start'
i = 0;
iMax = 20;
beaconSize = 0;

while toc < maxTime
    i = i + 1;
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    % get relevant data
    [sonar,beacon, bump] = newData(dataStore,beaconSize);
    
    %put data in comparable form
    [measurements, sonars, ARs ] = conditionSensors(sonar, beacon);
    %predict measurement
    predictMeas = hBeaconSonar(dataStore.truthPose(end,2:4),ARs,sonars,...
        map,beaconLoc,cameraR,sonarR);
    
    %store stuff
   
    if isempty(ARs)
        ARs = 0;
    end
    ARs = padarray(ARs,[0 2-size(ARs,2)],'post');
    dataStore.ARs = [dataStore.ARs;ARs];
    sonars = padarray(sonars, [0 3-size(sonars,2)],'post');
    dataStore.sonars = [dataStore.sonars;sonars];
    %TODO pad measurements with zeros so you can store it.
    measurements = padarray(measurements, [0 7-size(measurements,2)],'post');
    dataStore.measurements = [dataStore.measurements;measurements];
    predictMeas = padarray(predictMeas, [0 7-size(predictMeas,2)],'post');
    dataStore.predictMeas = [dataStore.predictMeas; predictMeas];
    %make sure that beacon datastore keeps up with everybody else
    beaconSize = size(dataStore.beacon,1);
    % CONTROL FUNCTION (send robot commands)
    if i == iMax %or confident of location TODO
        SetFwdVelAngVelCreate(CreatePort, 0, 0);
        break;
    else
        cmdV = slowV;
        cmdW = slowV/wheel2center;
    end
    
    % move forward if not bumped
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV,wheel2center);
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    
    pause(0.1);
    SetFwdVelAngVelCreate(CreatePort, 0, 0);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

end

function [sonar,beacon, bump] = newData(dataStore,beaconSize)
%newData extracts the most recent data from the data structure, grabs new
%beacon information 
sonar = dataStore.sonar(end,2:4);
bump = max(dataStore.bump(end,2:end));
beacon = [];
if size(dataStore.beacon,1) ~= beaconSize
numBeacons = size(dataStore.beacon,1)-beaconSize;
beacon = dataStore.beacon(end-numBeacons+1:end,2:end);
end

end