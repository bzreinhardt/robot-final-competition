function [exp] = hBeaconSonar(pose,beaconNums,sonars,map,beaconLoc,cameraRad,sonarRad)
%%
%hBeacon finds the expected beacon data for a series of robot poses based
%on a list of beacons

% ASSUMPTIONS 
% Assume that this function will only be used for positive identification
% Assume the robot can only see one beacon at a time
% 
% INPUTS
% pose - robot pose 3x1 or 1x3
% beaconNums - tag #'s of beacons in question
% 
% OUTPUTS
% exp

%% constants 

camViewAngle = 2*pi/3; %view angle on a single side of the camera
camViewDist = 2; %maximum distance the camera can see in meters
maxRange = 2.6;


exp = [];
if size(pose,1) ~= 3
    pose = pose';
end

for j = 1:length(sonars)
ang = (pi/2*sonars(j)-pi); 
expSonar = hSonar(pose, map, sonarRad, ang,maxRange);

exp = [exp,expSonar];
end
for i = 1:length(beaconNums)
    %hBeacon gives [cameraZ; cameraX] which is [beaconX,beaconY] in robot
    %coords
expBeacon = hBeacon(pose,beaconNums(i),beaconLoc,map,cameraRad); %hBeacon gives
exp = [exp,expBeacon'];
end




