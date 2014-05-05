function[VelX,VelW] = updateVW(wayPoints, gotopt, currPose)
% INPUTS
%   wapPoints - list of global frame x,y coordinates in an mx2 matrix
%   gotopt - the index describing which waypoint to go to
%   currPose - current pose of the robot (a 1x3 matrix [xposition
%   yposition, theta]
% OUTPUTS
%   VelX - forward velocity in m/s
%   VelW - angular velocity in rad/s

nextPt = wayPoints(gotopt,:);
waypointX = nextPt(1);
waypointY = nextPt(2);
currX = currPose(end,2);
currY = currPose(end,3);
theta = currPose(end,4);
xDist = waypointX - currX;
yDist = waypointY - currY;
distance = sqrt((xDist*xDist) + (yDist*yDist));
epsilon = 0.2;
[VelX,VelW] = feedbackLin(xDist, yDist, theta, epsilon);
