function [relocParts] = beaconRelocalize(G_beacon,C_beacon,M,mapLims,cameraR)
% sets up particles based on a seen beacon
%
% G_beacon - location of the beacon in global coordinates
% C_ beacon - location of the bacon in camera coordinates [C_x;C_y]
% (actually cameraz, camerax)
% M - number of particles
% mapLims - [minX, minY, maxX maxY]
% Camera R, location of the camera along the X coordinate of the robot

% fix the coordinates if in lab
global inLab
if inLab == 1
    C_beacon(2) = -C_beacon(2);
end

%set up loop constants
dtheta = 2*pi/M;
relocParts = zeros(3,M);
m = 1;
theta = 0;
%convert camera coordinates to robot coordinates. Assume camera is on X
%axis of robot
R_beacon = C_beacon + [0;cameraR];

while m <= M
    G_robot = G_beacon-[cos(theta) -sin(theta); sin(theta) cos(theta)]*...
        R_beacon;
    %if the robot pose is on the map, add it to the particle set
    if (G_robot(1) > mapLims(1) && G_robot(1) < mapLims(3) && ...
            G_robot(2) > mapLims(2) && G_robot(2) < mapLims(4))
        relocParts(:,m) = [G_robot;theta];
        theta = mod(theta + dtheta,2*pi);
        m = m + 1;
    else
        theta = mod(theta + dtheta,2*pi);
        
    end
end
end