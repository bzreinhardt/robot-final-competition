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
                   'predictMeas',[],'particles',[],'pfvar',[],...
                   'X',[]);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%% set up constraints
% maxTime = 500;  % Max time to allow the program to run (s)
maxV = 0.3;     % Max allowable forward velocity with no angular 
wheel2center = 0.13;  
sonarR = 0.16;
cameraR = 0;
slowV = 0.05;
% LOAD THE MAP - CHANGE THIS TO THE MAP IMPORTANT
load('ExampleMap2_2014.mat');
%% INITIALIZE PARTICLE FILTER
%4x1 row vector - [xMin yMin xMax yMax]
mapLims = [min([map(:,1);map(:,3)]),min([map(:,2);map(:,4)]),...
    max([map(:,1);map(:,3)]),max([map(:,2);map(:,4)])];
angles = [0:pi/4:(2*pi-pi/4)];
X_0 = initializePF(waypoints,angles);
%number of particles
M = size(X_0,2);
cameraR = 0;
sonarR = 0.16;
Q_sonar = 0.01;
cov_AR = 0.005;
R = [0.2 0 0; 0 0.2 0; 0 0 0.2];
theta = 0.1;
%rotM = [cos(theta) -sin(theta);sin(theta) cos(theta)];
Q_AR = cov_AR*eye(2);
%function to predict measurement based on map

%function to predict position based on odometry data
g = @integrateOdom;
h = @(X,ARs,sonars)hBeaconSonar(X,ARs,sonars,map,beaconLoc,cameraR,sonarR);
%noise functions
normNoise = @()normStateNoise([3,M],R);
%resampling functions
%lowVarResample = @lowVarSample;
resampleFcn = @noResample;
                        
%% Initiallize loop variables
tic
start = 'start'
%counter keeping track of number of loop iterations
i = 0;
iMaxLoc = 40;
% counter keeping track of beacon sightings
beaconSize = 0;
% boolean keeping track of whether you have seen a beacon
beaconSeen = 0;
% keep track of initial turning odometry
distTurned = 0;
%number of beacon sightings necessary for confident localization
minBeacons = 5;
%% plot the map
figure(1);clf; 
wallVisualizer(map, optWalls, beaconLoc, waypoints, ECwaypoints)
xlim([mapLims(1),mapLims(3)]);ylim([mapLims(2),mapLims(4)]);

%% Main loop
while toc < maxTime
    i = i + 1;
    if i == 1
        X_in = X_0;
    else
        X_in = X_out;
        delete(parts);
        delete(guess);
    end
    %% READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    %keep track of approximate distance turned 
    distTurned = distTurned + dataStore.odometry(end,3);
    % get relevant data
    [sonar,beacon, bump] = newData(dataStore,beaconSize);
    beaconSize = beaconSize + length(beacon);
    
    
    %switch resampling function when you see a beacon for the first time
    if beaconSize > 0 && beaconSeen == 0
        resampleFcn = @lowVarSample;
        beaconSeen = 1;
        
    end
        
    %put data in comparable form
    [measurements, sonars, ARs ] = conditionSensors(sonar, beacon);
    %predict measurement
    predictMeas = hBeaconSonar(dataStore.truthPose(end,2:4),ARs,sonars,...
        map,beaconLoc,cameraR,sonarR);
    %% RUN PARTICLE FILTER
    p_z = @(X,z)pfWeightSonarAR(X,z,h,ARs,sonars,Q_sonar,Q_AR,mapLims);
    [X_out,w_out] =  particleFilter(X_in,measurements,dataStore.odometry(end,2:3)',...
        g,p_z,normNoise,resampleFcn);
    %prune the output
    
    [X,locErr] = testConfidence(X_out,w_out,measurements,predictMeas,sonars,ARs);
    
    if locErr == 1
        disp('AR is dumb');
    end
    
    %% Draw stuff
    col = 'k';
    %figure(1);
    %xlim([-5 5]);ylim([-5 5]);
    %robot = circle(X_true(1:2),sonarR,10);
    
    guess = quiver(X(1),X(2),cos(X(3)),sin(X(3)));
    parts = quiver(X_out(1,:),X_out(2,:),w_out.*cos(X_out(3,:)),w_out.*sin(X_out(3,:)));
    set(parts,'color',col);
    set(guess,'color','r');
    xlim([mapLims(1),mapLims(3)]);ylim([mapLims(2),mapLims(4)]);
    drawnow;
    
    
    %% store stuff
   
    if isempty(ARs)
        ARs = 0;
    end
    if size(ARs,2)>5
        warning('what?')
    end
    ARs = padarray(ARs,[0 5-size(ARs,2)],'post');
    dataStore.ARs = [dataStore.ARs;ARs];
    sonars = padarray(sonars, [0 3-size(sonars,2)],'post');
    dataStore.sonars = [dataStore.sonars;sonars];
    %TODO pad measurements with zeros so you can store it.
    measurements = padarray(measurements, [0 13-size(measurements,2)],'post');
    dataStore.measurements = [dataStore.measurements;measurements];
    predictMeas = padarray(predictMeas, [0 13-size(predictMeas,2)],'post');
    dataStore.predictMeas = [dataStore.predictMeas; predictMeas];
    %make sure that beacon datastore keeps up with everybody else
    beaconSize = size(dataStore.beacon,1);
    %
    dataStore.particles = [dataStore.particles; X_out;w_out];
%    dataStore.pfvar = [dataStore.pfvar;X_var];
    dataStore.X = [dataStore.X;X'];
    
    
    %% CONTROL FUNCTION (send robot commands)
%     if i == iMaxLoc %or confident of location TODO
%         SetFwdVelAngVelCreate(CreatePort, 0, 0);
%         delete(parts);
%         delete(guess);
%         break;
%     else
        cmdV = slowV;
        cmdW = slowV/wheel2center;
%     end
    
    % move forward if not bumped
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    elseif beaconSize < 4 && distTurned < 2*pi
        % if you haven't seen enough beacons, keep turning in a cirlce
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV,wheel2center);
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    elseif beaconSize < 4 && distTurned > 2*pi
        % if you haven't seen enough beacons, and have turned in a circle,
        % follow a wall
        SetFwdVelAngVelCreate(CreatePort, 0, 0);
        
        break;
    else 
        %assume you're localized and follow a wall
        SetFwdVelAngVelCreate(CreatePort, 0, 0);
        
        break;
    end
    
    pause(0.1);
    
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