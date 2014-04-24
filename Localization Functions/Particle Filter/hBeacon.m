function [exp] = hBeacon(pose,beaconNums,beaconLoc,robotRad)
%%
%hBeaconSonar finds the expected beacon data for a series of robot poses based
%on a list of beacons

% ASSUMPTIONS 
% Assume that this function will only be used for positive identification
% Assume the robot can only see one beacon at a time
% 
% INPUTS
% pose - robot pose 3x1 or 1x3
% beaconNums - tag numbers of beacons in question 
% beaconLoc - locations of beacons on the map [Nx3] - 
% 
% OUTPUTS
% expBeacons - expected camera readings [z_camera;-x_camera] 

%% constants 
camViewAngle = 2*pi/3; %view angle on a single side of the camera
camViewDist = 2; %maximum distance the camera can see in meters

if size(pose,1) ~= 3
    pose = pose';
end

beacon = [];
for i = 1:length(beaconNums)
    beacon = [beacon; beaconLoc(beaconLoc(:,1) == beaconNums(i),:)];
end

%find beacons in robot pose

%transition matrix to robot pose
R_Q_G = [cos(pose(3)) sin(pose(3));  -sin(pose(3)) cos(pose(3))];

%beacons in the camera frame
C_beacons = R_Q_G*(beacon(:,2:3)'-pose(1:2))-[robotRad;0];

% %check if the robot would be able to see the beacons
% norms = sqrt(sum(C_beacons.^2,1));
% angles = atan2(C_beacons(2,:),C_beacons(1,:));
% expBeacons = [];
% for i = 1:size(beacons,1)
%     if norms(i) < camViewDist && abs(angles(i))<camViewAngle
%         expBeacons = [expBeacons; beacons(i,:);
%     end

%convert C_beacons to measurement values

exp = C_beacons;
end



