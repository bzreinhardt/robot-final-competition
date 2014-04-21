function[dataStore] = motionControl(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
% ODOMTEST: commands robot to drive in a curve until it hits a wall, then
% back up for two time steps and stop
% 
%   dataStore = odomTest(CreatePort,SonarPort,BeaconPort,tagNum,maxTime) 
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
                   'beacon', [], ...
                   'deadReck',[], ...
                   'ekfMu',[], ...
                   'ekfSigma',[],...
                   'GPS',[],...
                   'particles',[]);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%set up constraints
 maxDuration= 100;  % Max time to allow the program to run (s)
 maxDistSansBump= 5; % Max distance to travel without obstacles (m)
 maxV = 0.49;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
 maxVelIncr= 0.005;  % Max incrementation of forward velocity (m/s)
 maxSonarRange = 3;
 robotRadius = 0.16; 
 Wheel2center = 0.13 ;
 sonarAngles = [-pi/2,0,pi/2];
 
 R = 0.01*eye(3); %R = 3x3 because of 3 state variables
 Q = 0.001 * eye(3); %Q = 3x3 because of 3 sonar sensors
 
 g = @integrateOdom;
 
 G = @GjacDiffDrive;
 
 
%  %GPS STUFF
%  h_gps = @hGPS;
%  H_gps = @HjacGPS;
%SONAR STUFF
load('cornerMap.mat');
h_sonar_all = @(robotPose)sonarPredict(robotPose,cornerMap,robotRadius,...
    sonarAngles,maxSonarRange);
H_sonar_all = @(X)Hjac3Sonar(X,cornerMap,robotRadius,...
    sonarAngles,maxSonarRange);

p_z = @(X,z)pfWeightSonar(X,z,h_sonar_all,Q);
    

                        
% Initiallize loop variables
tic
start = 'start'
v = 1;% Forward velocity (m/s)
w = 0.5; %angular velocity (m/s)


sigma0 = [2 0 0; 0 2 0; 0 0 0.1]; %initial covariance matrix
% sigma0 = [4 0 0 ; 0 4 0; 0 0 0.02]; %bigger covariance matrix
bumped = 0;
stepsSinceBump = 0;

%initialize sensordata
[noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
%initialize dead reckoning with truth pose data
dataStore.deadReck = [dataStore.deadReck;dataStore.truthPose(1,:)];

%EKF STUFF
%initialize ekfMu with truth pose data
%dataStore.ekfMu = [dataStore.ekfMu;dataStore.truthPose(1,:)];
%initialize ekfMU with less truthy data
%mu0 = [-2;-1.5;pi/2];
%dataStore.ekfMu = [dataStore.ekfMu;toc mu0'];

%EKF STUFF
%only keep sigmas in ekfSigma, no time - just use mu times
%dataStore.ekfSigma = [dataStore.ekfSigma; sigma0];
% GPS STUFF
% %noisy GPS data 
% newGPS = updateGPS(dataStore,Q);
% dataStore.GPS = [dataStore.GPS; newGPS];

%PF STUFF
%initialize particles for PF
M = 20; 
x0 = -5*rand(1,M);
y0 = 5-10*rand(1,M);
theta0 = 0.2 - 0.4*rand(1,M);
p0 = [x0;y0;theta0];
w0 = ones(1,M)/M;
t = toc;
%the particles structure will store weights that apply to the PREVIOUS set
%of states
dataStore.particles = [dataStore.particles; t w0; [t;t;t] p0];
     
%plot stuff
% figure(1);clf;
% plotWalls(cornerMap);
% hold on;
% plot(p0(1,:),p0(2,:),'x');
while toc < maxTime
    
%     %test beacon
%     bacon = BeaconPort
%     test = CameraSensorCreate(BeaconPort)
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    
    deadReckPose = integrateOdom(dataStore.deadReck(end,2:4)',...
        dataStore.odometry(end,2),dataStore.odometry(end,3));
    
    dataStore.deadReck = [dataStore.deadReck;...
        dataStore.odometry(end,1) deadReckPose'];
    
% GPS STUFF    
%     newGPS = updateGPS(dataStore,Q);
%     dataStore.GPS = [dataStore.GPS; newGPS];
%     %LOCALIZATION
%     mu0 = dataStore.ekfMu(end,2:4)';
%     %EKF STUFF
% %     sigma0 = dataStore.ekfSigma(end-2:end,:); 
%     u = dataStore.odometry(end,2:3)';
%     
% %     %GPS STUFF
% %     z = newGPS(2:4)'; 
% %     [mu, sigma] = extendedKalmanFilter(mu0,sigma0,z,u, ...
% %                                         g,h_gps, G, H_gps, Q, R);
% =======================
%%SONAR STUFF
% % % need to find NAN measurements
% sonarMeas = dataStore.sonar(end,2:4);
% expectSonarMeas = h_sonar_all(mu0);
% goodMeas = [];
% %if both the sonar measurement and expected sonar measurement are good, use
% %that sensor.
% for i = 1:length(sonarMeas)
%     if ~isnan(sonarMeas(i)) && ~isnan(expectSonarMeas(i))
%         goodMeas = [goodMeas, i];
%         Q_good = Q(i,i);
%     end
% end
% Q_good = diag(Q_good);
% h_sonar = @(robotPose)sonarPredict(robotPose,cornerMap,robotRadius,...
%     sonarAngles(goodMeas),maxSonarRange);
% H_sonar = @(X)Hjac3Sonar(X,cornerMap,robotRadius,...
%     sonarAngles(goodMeas),maxSonarRange);
% z = sonarMeas(goodMeas)';
%
% [mu, sigma] = extendedKalmanFilter(mu0,sigma0,z,u, ...
%                                       g,h_sonar, G, H_sonar, Q_good, R);
%         
%         
%    ===============================  
%     %store the new EKF data
%     dataStore.ekfMu = [dataStore.ekfMu;toc mu'];
%     dataStore.ekfSigma = [dataStore.ekfSigma;sigma];

%PF STUFF 
X_in = dataStore.particles(end-2:end,end-M+1:end);
z = dataStore.sonar(end,2:4);
u = dataStore.odometry(end,2:3)';
[X_out,w_out] = particleFilter(X_in,z,u,g,p_z);
t = toc;
dataStore.particles = [dataStore.particles; t w_out; [t;t;t] X_out]; 
%plot stuff
% figure(1);clf;
% plotWalls(cornerMap);
% hold on;
% plot(X_out(1,:),X_out(2,:),'x');
    % CONTROL FUNCTION (send robot commands)
    
    % Check for and react to bump sensor readings
  
    [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster, ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(CreatePort);
    
    if BumpRight || BumpLeft || BumpFront;
        bumped = 1;
        v = -v;
        w =0;
         
    end
    if bumped
        stepsSinceBump = stepsSinceBump + 1;
    end
    if stepsSinceBump > 2
        v = 0;
        w = 0;
    end
        
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);

    else
        [cmdV, cmdW] = limitCmds(v, w, maxV,Wheel2center);
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );

    end
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

end

function newGPS = updateGPS(dataStore,Q)
    noise = Q*randn(3,1);
    newGPS = [dataStore.truthPose(end,1), ...
        dataStore.truthPose(end,2:4) + noise'];
end


