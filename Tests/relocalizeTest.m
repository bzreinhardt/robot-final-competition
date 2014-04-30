function[dataStore] = relocalizeTest(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
% relocalizeTest: test relocalization function  
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
                   'predictMeasTrue',[],'predictMeasGuess',[],...
                   'particles',[],'pfvar',[],...
                   'X',[],'mu',[],'sigma',[]);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%% set up constraints
% maxTime = 500;  % Max time to allow the program to run (s)
%check whether you are in the lab or not 
global inLab;
maxV = 0.3;     % Max allowable forward velocity with no angular 
wheel2center = 0.13;  
sonarR = 0.16;
cameraR = 0;
if inLab == 1
    load('ExampleLabMap_2014.mat');
    cameraR = 0.15;
else
    load('ExampleMap2_2014.mat');
end
slowV = 0.05;
%TODO load covariance matrix

%% INITIALIZE PARTICLE FILTER
mapLims = [min([map(:,1);map(:,3)]),min([map(:,2);map(:,4)]),...
    max([map(:,1);map(:,3)]),max([map(:,2);map(:,4)])];
angles = [0:pi/4:(2*pi-pi/4)];
X_0 = initializePF(waypoints,angles);

%number of particles
M = size(X_0,2);

Q_sonar = 0.01;
cov_AR = 0.005;
R = 0.2*eye(3);
theta = 0.1;
%rotM = [cos(theta) -sin(theta);sin(theta) cos(theta)];
Q_AR = cov_AR*eye(2);
%function to predict measurement based on map

%function to predict position based on odometry data
G = @GjacDiffDrive;
g = @integrateOdom;
h = @(X,ARs,sonars)hBeaconSonar(X,ARs,sonars,map,beaconLoc,cameraR,sonarR);
H = @HjacSonarAR;
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

% counter keeping track of beacon sightings
beaconSize = 0;
% boolean keeping track of whether you have seen a beacon
beaconSeen = 0;
% keep track of initial turning odometry
distTurned = 0;
%number of beacon sightings necessary for confident localization
minBeacons = 5;
% code keeping track of localization events
locEvent = 0;
% keep track of erroneous measurements and good measurements
badMeas = 0;
goodMeas = 0;
beaconSeenIndex = 0;
localized = 0; %keep track of whether you are confident of your position
initLoc = 0; %keep track of whether you have figured out where you are initially
resamp = 0;  %keep track of whether you should resample

if inLab == 1
figure(1);clf; 
wallVisualizer(map, optWalls, beaconLoc, waypoints, ECwaypoints)
xlim([mapLims(1),mapLims(3)]);ylim([mapLims(2),mapLims(4)]);    
end


%% Main loop
while toc < maxTime
    
    %% READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    
    %Test for localizing
    %REMOVE THIS
    X_0 = dataStore.truthPose(end,2:4)'*ones(1,M);
    %% Loop Housekeeping
    i = i + 1;
    if i == 1
        %change me
        X_in = X_0;
        mu = [0;0;0];
        sigma = 0.2*eye(3);
    else
        X_in = X_out;
%        delete(parts);
        delete(guess);
%        delete(muGuess);
        delete(PFGuess);
%        delete(covE);
        if inLab == 1
            delete(robot);
            delete(truth);
        end
    end
    %% Condition data
    
    %keep track of approximate distance turned 
    distTurned = distTurned + dataStore.odometry(end,3);
    % get relevant data
    [sonar,beacon, bump] = newData(dataStore,beaconSize);
    beaconSize = beaconSize + length(beacon);
    
    
  
        
    %put data in comparable form
    [measurements, sonars, ARs ] = conditionSensors(sonar, beacon);
    %predict measurement
    predictMeasTrue = hBeaconSonar(dataStore.truthPose(end,2:4),ARs,sonars,...
        map,beaconLoc,cameraR,sonarR);
    
    %% RUN PARTICLE FILTER
    
    p_z = @(X,z)pfWeightSonarAR(X,z,h,ARs,sonars,Q_sonar,Q_AR,mapLims);
    u = dataStore.odometry(end,2:3)';
    [X_out,w_out] =  particleFilter(X_in,measurements,u,...
        g,p_z,normNoise,resampleFcn);
    %prune the output
    X_PF = genGuess(X_out,w_out);
   
%     %% RUN KALMAN FILTER
%     [mu,sigma] = EKF(mu,sigma,measurements, u,g,h, G, H, ...
%         Q_sonar,Q_AR, R,sonars,ARs);
    %% Test for walls, errors, waypoints
    
    
    [locEventPF,predictMeasGuess] = testConfidence(X_PF,measurements,h,sonars,ARs);
 %   [locEventEKF,predictMeasGuess] = testConfidence(mu,measurements,h,sonars,ARs);
    %different localization situations
  %  if localized == 1 
 %   locEvent = locEventEKF;
 %   X = mu;
    %
 %   else
        X = X_PF;
        locEvent = locEventPF;
  %  end
    %switch resampling function when you see a beacon for the first time
    if beaconSize > 0 && beaconSeen == 0
        resampleFcn = @lowVarSample;
        beaconSeen = 1; 
        %mark when you first see a beacon
        beaconSeenIndex = i;
        
    end
    %if you see a beacon and aren't localized resample  
    if numel(ARs) > 0 && localized == 0 
        resamp = resamp - 1;
        if resamp < 1
            beaconNum = ARs(1);
            G_beacon = beaconLoc(beaconLoc(:,1) == beaconNum,2:3)';
            C_beacon = measurements(length(sonars)+1:length(sonars)+2)';
            X_out = beaconRelocalize(G_beacon,C_beacon,M,mapLims,cameraR);
            resamp = 2;
        end
    end
    % determine that you're localized if you've got 3 good measurements in
    % a row
    if goodMeas >= 3
        localized = 1;
        if initLoc == 0
            initLoc = 1;
            disp('found myself for the first time')
        end
 %       mu = X;
    elseif badMeas >= 3;
        localized = 0;
    end
    disp(localized)
    
    if locEvent == 1
        badMeas = badMeas +1;
        goodMeas = 0;
        if badMeas > 3
            disp('I dont know where I am');
 %           X_out = relocalize(mu,sigma,M);
            reloc_i = 0;
        end
        guessCol = 'r';
            
    else
        goodMeas = goodMeas + 1;
        badMeas = 0;
        lastGoodMeas = X;
        mu = X;
        guessCol = 'g';
    end
    
    %% Draw stuff
    col = 'k';
   
    PFGuess = quiver(X_PF(1),X_PF(2),cos(X_PF(3)),sin(X_PF(3)));
%    muGuess = quiver(mu(1),mu(2),cos(mu(3)),sin(mu(3)));
    guess = quiver(X(1),X(2),cos(X(3)),sin(X(3)));
%    parts = quiver(X_out(1,:),X_out(2,:),w_out.*cos(X_out(3,:)),w_out.*sin(X_out(3,:)));
%   set(muGuess,'color','k');
    set(guess,'color',guessCol);
    set(PFGuess,'color','b');
    if inLab == 1
        xlim([mapLims(1),mapLims(3)]);ylim([mapLims(2),mapLims(4)]);
        robot = circle(X_true(1:2),sonarR,10);
        truth = quiver(X_true(1),X_true(2),cos(X_true(3)),sin(X_true(3)));
        set(truth,'color','g');
        covE = plotCovEllipse(mu(1:2),sigma(1:2,1:2)); 
    end
    %disp(sigma);
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
    predictMeasTrue = padarray(predictMeasTrue, [0 13-size(predictMeasTrue,2)],'post');
    dataStore.predictMeasTrue = [dataStore.predictMeasTrue; predictMeasTrue];
    
    predictMeasGuess = padarray(predictMeasGuess, [0 13-size(predictMeasGuess,2)],'post');
    dataStore.predictMeasGuess = [dataStore.predictMeasGuess; predictMeasGuess];
    %make sure that beacon datastore keeps up with everybody else
    beaconSize = size(dataStore.beacon,1);
    %
    dataStore.particles = [dataStore.particles; X_out;w_out];
%    dataStore.pfvar = [dataStore.pfvar;X_var];
    dataStore.X = [dataStore.X;X'];
    dataStore.mu = [dataStore.mu;mu'];
    dataStore.sigma = [dataStore.sigma;sigma];
    
    
    %% CONTROL FUNCTION (send robot commands)
%     if i == iMaxLoc %or confident of location TODO
%         SetFwdVelAngVelCreate(CreatePort, 0, 0);
%         delete(parts);
%         delete(guess);
%         break;
%     else
if localized == 0
    % if you lose loclaization, stop and then turn in a cirlce
    
        cmdV = 0;
        cmdW = slowV/wheel2center;

    
elseif locEvent == 2
        % if you see a wall try to bump into the wall
elseif max(dataStore.bump(end,2:end)) == 1
%if you bump into a wall, back up 
travelDist(CreatePort, slowV, -0.1);

elseif localized == 1
    %stop
    cmdV = slowV;
        cmdW = 0;
        
else 
   cmdV = 0;
        cmdW = 0;
 end
    
    % move forward if not bumped
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
        
    
    else 
        %assume you're localized and follow a wall
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
        
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