function[range] = hSonar(robotPose,map,robotRad,angles,maxRange)
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
if size(robotPose,1) == 1
    robotPose = robotPose';
end

G_Q_R = [cos(robotPose(3)), -sin(robotPose(3)); ...
            sin(robotPose(3)), cos(robotPose(3))]; %rotation matrix from robot frame to global frame

inf = maxRange + 1; %make sure our infinte vector goes past the max range
%do we assume the sonar detectors are on the outer edge of the robot?
range = NaN*ones(length(angles),1); %initialize range matrix
for i = 1:length(angles)
    %cycle through the sensors
    %assume sensor is mounted at angle on radius of robot
    pos_s_R = [robotRad*cos(angles(i));robotRad*sin(angles(i))]; %position of the sensor in robots frame
    pos_s_G = G_Q_R*pos_s_R+robotPose(1:2);
    %generate the endpoints of the line projected from the sensor
    pos_inf = pos_s_G + inf *[cos(angles(i)+robotPose(3));sin(angles(i)+robotPose(3))]; 
    %check whether each wall intersects the sensor line
    for k = 1:length(map(:,1)) %check all the walls
    %check for intersections
    [isect,x,y,ua]= intersectPoint(pos_s_G(1),pos_s_G(2),pos_inf(1),pos_inf(2),...
        map(k,1),map(k,2),map(k,3),map(k,4));
     if isect %if there's an intersection w/ wall calculate distance to wall
         dist = ((x-pos_s_G(1)).^2+(y-pos_s_G(2)).^2).^.5;
         if isnan(range(i)) || dist < range(i) 
             %if the distance to the wall is the shortest distance, set it
             %as the expected range measurement for that sensor
            range(i) = dist;
         end
     end
    end
end