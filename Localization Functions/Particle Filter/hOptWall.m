function[optWall] = hOptWall(pose,beaconNums,sonars,map,beaconLoc,cameraRad,sonarRad,optWalls,hBeaconSonar,meas,predMeas)
% HSONAR: predict the sonar range measurements for a robot operating
% in a known map.
% 
%   RANGE = SONARPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES,MAXRANGE) returns 
%   the expected sonar range measurements for a robot operating in a known 
%   map. 
% 
%   INPUTS
%       robotPose   3-by-1 pose vector in global coordinates (x,y,theta)
%       map         N-by-4 matrix containing the coordinates of walls in 
%                   the environment: [x1, y1, x2, y2]
%       robotRad    robot radius (meters)
%       angles      K-by-1 vector of the angular positions of the sonar
%                   sensor(s) in robot coordinates, where 0 points forward
%       maxRange    maximum sonar range (meters)
% 
%   OUTPUTS
%       range       K-by-1 vector of sonar ranges (meters)
% 
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework 3
%   Reinhardt, Benjamin
% add each optional wall to the map and compare the expected value to the
% actual value
predMeas = feval(hBeaconSonar,pose,beaconNums,sonars,map,beaconLoc,cameraRad,sonarRad);
currentError = sum(abs(meas - predMeas));
optWall = []; smallestError = currentError;

for i = 1:size(optWalls,1)
    %add wall to map
    testMap = [map;optWalls(i,:)];
    testPred = feval(hBeaconSonar,pose,beaconNums,sonars,testMap,beaconLoc,cameraRad,sonarRad);
    testError = sum(abs(meas - testPred));
    if testError < smallestError
        optWall = i;
        smallestErr = testError;
    end
end
end