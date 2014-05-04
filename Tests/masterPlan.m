function[dataStore] = masterPlan(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
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
                   'X',[],'mu',[],'sig',[]);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%% set up constraints
% maxTime = 500;  % Max time to allow the program to run (s)
%check whether you are in the lab or not 
global isLab;
global planOn;
global locOn;
maxV = 0.3;     % Max allowable forward velocity with no angular 
wheel2center = 0.13;  
sonarR = 0.16;
cameraR = 0;
robotRad = sonarR;
%tunable Parameters
closeEnough = sonarR;
if isLab == 0
    load('Map2 - Path1.mat');
    path = shortestPath(:,2:3);
end
if isLab == 1
    load('ExampleLabMap_2014.mat');
    cameraR = 0.13;
    sonarR = 0.15;
else
    load('ExampleMap2_2014.mat');
end
slowV = 0.05;
%TODO load covariance matrix
maxBadMeas = 5; %max number of bad measurements before losing localization

%% INITIALIZE PARTICLE FILTER
mapLims = [min([map(:,1);map(:,3)]),min([map(:,2);map(:,4)]),...
    max([map(:,1);map(:,3)]),max([map(:,2);map(:,4)])];


angles = [0:pi/5:(2*pi-pi/5)];
X_0 = initializePF(waypoints,angles);

%number of particles
M = size(X_0,2);

Q_sonar = 0.1;
cov_AR = 0.01;
R = 0.01*eye(3);
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
sig0 = 0.2*eye(3);
                        
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

% code keeping track of localization events
locEvent = 0;
% keep track of erroneous measurements and good measurements
badMeas = 0;
goodMeas = 0;
beaconSeenIndex = 0;
localized = 0; %keep track of whether you are confident of your position
initLoc = 0; %keep track of whether you have figured out where you are initially
resamp = 0;  %keep track of whether you should resample
checkWall = -1; %keep track of what wall needs to be checked based on current position
% waypoints left to visit
wayPtsUnvisited = [ waypoints;ECwaypoints];
% waypoints already visited
wayPtsVisited = [];
optWallFlag = 0;
truthSize = 0;
bumped = 0; % keep track of whether you bumped a wall

%% Draw initial stuff
if isLab == 1
figure(1);clf; 
wallVisualizer(map, optWalls, beaconLoc, waypoints, ECwaypoints) 

xlim([mapLims(1),mapLims(3)]);ylim([mapLims(2),mapLims(4)]);
drawnow;
end


%% PLANNING INITIALIZATION KEVIN's STUFF GOES HERE %%
if planOn ~= 0
if isLab == 1
    maxV = 0.1;
else
    maxV = 0.5;
end
buffer = robotRad;

walls = map;

wpOrWall = 1;

regWps = waypoints;
regWps = regWps.';
ecWps = ECwaypoints;
ecWps = ecWps.';
wps = [regWps ecWps];

unvisitedWPs = wps;
visitedWPs = [];

flag = 0;

closeEnough = 0.2;
gotopt = 1;
stop = 0;
dontMove = 0;
end

%% Main loop
while toc < maxTime
    %% Kevin Code 
    if planOn ~= 0
        
     if (isempty(unvisitedWPs) && isempty(optWalls))
         %if you've visited all the waypoints and found all the walls, stop
        SetFwdVelAngVelCreate(CreatePort,0,0);
        BeepRoomba(CreatePort);
        BeepRoomba(CreatePort);
        BeepRoomba(CreatePort);
         break
     end
    end
    %% READ & STORE SENSOR DATA
    toc 
    disp('start reading data');
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    toc
    disp('stop reading data');
    %% Loop Housekeeping
    i = i + 1;
    if i == 1
        %Test for localizing
        %     %REMOVE THIS
         X_0 = dataStore.truthPose(end,2:4)'*ones(1,M);
         localized = 1;
       initLoc = 1;
        %change me
        X_in = X_0;
        mu = mean(X_0,2);
        sig = sig0;
    else
        X_in = X_out;
        if isLab == 1;
        figure(1);
        end
        delete(guess); 
       
        delete(PFGuess);
       
        
  %      delete(parts);
        if isLab == 1
            delete(robot);
            delete(truth);
        end
    end
    %% Condition data
    u = dataStore.odometry(end,2:3)';
    %comment this out for real competition!
    if size(dataStore.truthPose,2) == truthSize
        %if truthPose didn't update, update it yourself with odom data
        newTrue = feval(g,dataStore.truthPose(end,2:4)',u);
        dataStore.truthPose = [dataStore.truthPose;[toc, newTrue']];
    else
        truthSize = size(dataStore.truthPose,2);
    end
    X_true = dataStore.truthPose(end,2:4)';
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
    %update h to the current map
    h = @(X,ARs,sonars)hBeaconSonar(X,ARs,sonars,map,beaconLoc,cameraR,sonarR);
    %update p_z to the current sensors
    p_z = @(X,z)pfWeightSonarAR(X,z,h,ARs,sonars,Q_sonar,Q_AR,mapLims);
    
    [X_out,w_out] =  particleFilter(X_in,measurements,u,...
        g,p_z,normNoise,resampleFcn);
    %prune the output
    if beaconSeen == 0 && i > 1
        %if you haven't switched resampling on, compound the weights
        w_out = w_out.*dataStore.particles(end,:);
    end
    X_PF = genGuess(X_out,w_out);
    [locEventPF,predictMeasGuess,wallEventPF] = testConfidence(X_PF,measurements,h,sonars,ARs);
    %% RUN KALMAN FILTER
    if initLoc == 1
        [mu,sig] = EKF(mu,sig,measurements, u,g,h, G, H, ...
            Q_sonar,Q_AR, R,sonars,ARs);
        [locEventEKF,predictMeasGuess,wallEvent] = testConfidence(mu,measurements,h,sonars,ARs);
    end
    %% Localization Algorithm - Determine Positio
    
    %If you're confident of your position, use an EKF
    %If you're localized, but have a sonar discrepency, check for optional
    %walls
    if localized == 1
        locEvent = locEventEKF;
%         if wallEvent == 2 || optWallFlag == 2
%             %a missing wall will screw up the EKF, so if you suspect a wall
%             %go completely on odometry
%             
%             checkWall = hOptWall(mu,ARs,sonars,map,beaconLoc,cameraR,...
%                 sonarR,optWalls,@hBeaconSonar,measurements,predictMeasGuess)
%            
%            
%             if checkWall > 0
%             optWallFlag = 2;
%              disp('I suspect there is a wall, using odometry only');
%              X = feval(g,X,u);
%             end
%             %CHANGE THIS MAYBE Things to do for wall checking
% %             if checkWall == optWall && optWall ~= 0
% %                 disp(strcat('Found wall # ',num2str(optWall)));
% %                 wallpt = [(optWalls(optWall,1)+optWalls(optWall,3))/2;
% %                     (optWalls(optWall,2)+optWalls(optWall,4))/2];
% %                 plot(wallpt(1),wallpt(2),'rx');
% %                 map = [map;optWalls(optWall,:)];
% %                 %need to change h function handle to have new map
%                
        
       
            
       
            X = mu;
        
        %if you're localized, check for waypoints
        [wayPtsVisited,wayPtsUnvisited,wayPtAlert] = checkWayPts(X,wayPtsVisited,wayPtsUnvisited,closeEnough);
        if wayPtAlert == 1
            %do stuff - replan, beep etc
            disp('WAYPOINT!');
            plot(wayPtsVisited(end,1),wayPtsVisited(end,2),'gx');
            wayPtAlert = 0;
        end
        
    %If you're not confident, use the particle filter    
    else
        X = X_PF;
        locEvent = locEventPF;

        
    end
   
 %% LOCALIZATION ALGORITHM - Determine confidence   
    
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
            %tunable parameter - number of steps to wait before resampling
            resamp = 10;
        end
    end
    % determine that you're localized if you've got 3 good measurements in
    % a row
    if goodMeas >= 3
        if localized == 0 && beaconSeen == 1
            disp('found myself');
            mu = X;
            sig = sig0;
            localized = 1;
        end
        if initLoc == 0 && beaconSeen == 1
            initLoc = 1;
            disp('found myself for the first time')
        end
       
    end
    
    %react to bad measurements
    if locEvent == 1
        %increment bad measurement counter if you're not in a possible
        %optional wall situation
        if optWallFlag ~=2
            badMeas = badMeas +1;
            goodMeas = 0;
        end
        
        %three bad measurements in a row means you're lost
        if badMeas > maxBadMeas && initLoc == 1
            localized = 0;
            disp('Lost myself');
            distTurned = 0;
        end
        guessCol = 'r';
        
    else
        goodMeas = goodMeas + 1;
        badMeas = 0;
        lastGoodMeas = X;
        guessCol = 'g';
    end
    disp(localized)
    
    
   %% PLANNING STUFF - KEVIN's CODE GOES HERE
   if planOn ~= 0
   currLoc = X;
   
   if (wpOrWall == 1) || (wpOrWall == 2)
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
   end
    %% Draw stuff
    if isLab == 1
    figure(1);
    end
     
    col = 'k';

    PFGuess = quiver(X_PF(1),X_PF(2),cos(X_PF(3)),sin(X_PF(3))); hold on

    guess = quiver(X(1),X(2),cos(X(3)),sin(X(3)));
   % parts = quiver(X_out(1,:),X_out(2,:),w_out.*cos(X_out(3,:)),w_out.*sin(X_out(3,:)));
%   set(muGuess,'color','k');
    set(guess,'color',guessCol);
    set(PFGuess,'color','b');
    
    if isLab == 1
        xlim([mapLims(1),mapLims(3)]);ylim([mapLims(2),mapLims(4)]);
        robot = circle(X_true(1:2),sonarR,10);
        truth = quiver(X_true(1),X_true(2),cos(X_true(3)),sin(X_true(3)));
        set(truth,'color','g');
    end

   
    %disp(sig);
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
    dataStore.sig = [dataStore.sig;sig];
    

    
    
    %% CONTROL FUNCTION (send robot commands)
%     if i == iMaxLoc %or confident of location TODO
%         SetFwdVelAngVelCreate(CreatePort, 0, 0);
%         delete(parts);
%         delete(guess);
%         break;
%     else
%IF you hit a wall, set bumped flag to 1
if max(dataStore.bump(end,2:end)) == 1
    bumped = 1;
    if dataStore.bump(end,2) == 1
        bumpSide = -pi/2;
    elseif dataStore.bump(end,3) == 1
        bumpSide = pi/2;
    else
        bumpSide = 0;
    end
    disp('hit a wall')
end
if localized == 0
    % LOCALIZATION CODE - BEN'sSTUFF
    %turn until you see a beacon
    if isempty(beacon) && distTurned < 2*pi && bumped == 0
    cmdV = 0;
    cmdW = slowV/wheel2center;
    
    
    disp('unlocalized and cannot see a beacon - disturned < 2pi');
    elseif isempty(beacon) && distTurned >2*pi && bumped == 0
        %move randomly?
        cmdV = 4 * slowV;
        cmdW = 0;
        disp ('unlocalized, turned in circle and still cannot see beacon disturned > 2pi');
    elseif ~isempty(beacon)
        distTurned = 0;
        cmdV = 0;
        cmdW = 0;
        disp('Unolocalized but can see a beacon');
        bumped = 0;
    elseif bumped > 0 && isempty(beacon)
        disp('unlocalized responding to bump. cant see a beacon.');
        if bumped < 4
            cmdV = -slowV;
            cmdW = 0;
            bumped = bumped + 1;
        elseif bumped >= 4 && bumped < 7
            cmdV = 0;
            if bumpSide == -pi/2 %turn to the left if bump is on the right
            cmdW = slowV/2;
            else
                cmdW = -slowV;
            end
            bumped = bumped + 1;
        elseif bumped >= 7
            bumped = 0;
            cmdV  = 4 * slowV;
            cmdW = 0;
        end
        
    end
    
    
elseif localized == 1
    if planOn == 0
        %do backup bump
        if max(dataStore.bump(end,2:end)) == 1
            bumped = 1;
            if dataStore.bump(end,2) == 1
                bumpSide = -pi/2;
            elseif dataStore.bump(end,3) == 1
                bumpSide = pi/2;
            else
                bumpSide = 0;
            end
            disp('hit a wall')
        end
        if bumped == 0
            cmdV = 2*slowV;
            cmdW = 0;
            disp('moving while localized');
        elseif bumped > 0
            disp('responding to bump while localized');
            if bumped < 4
                cmdV = -slowV;
                cmdW = 0;
                bumped = bumped + 1;
            elseif bumped >= 4 && bumped < 7
                cmdV = 0;
                if bumpSide == -pi/2 %turn to the left if bump is on the right
                    cmdW = slowV/2;
                else
                    cmdW = -slowV/2;
                end
                bumped = bumped + 1;
            elseif bumped >= 7
                bumped = 0;
                cmdV  = 4 * slowV;
                cmdW = 0;
            end
        end
        
    % CODE ASSUMING YOU KNOW WHERE YOU ARE - KEVIN's STUFF GOES HERE
    elseif planOn ~=0
        if stop == 0
            currPose = [toc;X]';
            
            if (wpOrWall == 4) && ((gotopt == m) || (gotopt == (m - 1))) && ((dataStore.bump(end,2) == 1) || (dataStore.bump(end,3) == 1) || (dataStore.bump(end,7) == 1))
                SetFwdVelAngVelCreate(CreatePort, 0,0 );
                dontMove = 1;
                walls = [walls; optWalls(removeIdx,:)];
                optWalls(removeIdx,:) = [];
                wpOrWall = 1;
%             end
%             elseif ((sum(dataStore.bump((end - 5):end,2)) > 3) || (sum(dataStore.bump((end - 5):end,3)) > 3) || (sum(dataStore.bump((end - 5):end,7)) > 3))
                %% here we handle the case when the closest point goes through an optional wall
                % 
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
%                     SetFwdVelAngVelCreate(CreatePort, 0,0 );
                 
                    dontMove = 1;
                    %return
                end
            end
            if (dontMove == 0 )
            [velX, velW] = updateVW(waypoints, gotopt, currPose);
            [cmdV,cmdW] = limitCmds(velX,velW,maxV,robotRad);
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
            end
        end
          if max(dataStore.bump(end,2:end)) == 1
        %if you bump into a wall, back up
        travelDist(CreatePort, slowV, -0.2);
        turnAngle(CreatePort, slowV, 30);
        disp('hit a wall');
          end
    end
end
 if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
 else
     SetFwdVelAngVelCreate(CreatePort,cmdV,cmdW);
 end
    % move forward if not bumped
    
    % if overhead localization loses the robot for too long, stop it
 
    
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

    function [cmdV,cmdW,done] = backupBump(turnDist, moveDist)
    end

end

function [cmdV, cmdW,bumped] = bumpResponse(bumped)
end