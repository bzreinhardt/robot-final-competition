function [measurements, sonars, ARs ] = conditionSensors(sonarData, cameraData)
%CONDITIONSENSORS eliminates bad/useless sensor data and composes 
% the camera and sonar data into one set of useful measurements 
 % INPUTS
% sonarData - data from the sonar sensors [1x3]
% cameraData - data from the AR sensors [Nx5] - 
%   N is the number of seen beacons (can be zero)

%OUTPUTS
% % measurements - row vector of useful measurements - 
% %     1 distance measurement for each valid sonar and 2 for each camera [1xM] 
% % sonars - row vector of sonars that provided valid measurements [1xS] sonars
% %     are numbered from 1 to 3 going counterclockwise from the one located at 
% %     -pi/2 - R = 1, F = 2, L = 3. order corresponds to the range data in measurements.
% % ARs - row vector of tag numbers for the seen AR tags, order corresponds to 
% %     order of AR data in measurments
sonars = [];
measurements = [];
ARs = [];
%find non-NaN sonar measurements
for i = 1:3
    if ~isnan(sonarData(i))
        measurements = [measurements,sonarData(i)];
        sonars = [sonars, i];
    end
end
%find useful beacon measurements - put them in form of X,Y in robot
%coordinates
for j = 1:size(cameraData,1)
    measurements = [measurements,cameraData(j,5),cameraData(j,3)];
    ARs = [ARs, cameraData(j,1)]; 
end
end

