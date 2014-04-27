function [w] = pfWeightSonarAR(X,z,h,ARs,sonars,Q_sonar,Q_AR,mapLims)
% : Find the weight of a state particle by comparing the sensor
% measurements expected from that particle with actual measurements
% 
%   [w] = pfWeightSonar(X,z,h,Q) takes a particle state, finds the expected
%   measurements at that state, evaluates the gaussian pdf with a mean of
%   the expected measurement and the measurement covariance at that point 
%   and uses that value as the point's weight.
% 
%   INPUTS
%       X -  particle state 3x1
%       h  - expected measurement function handle - takes the state as an
%           argument
%       z  - actual measurement 1xK or Kx1 where k is the number of sensors
%       Q_sonar - Variance of a single sonar sensor
%       Q_AR - covariance matrix for X/Y measurements of a AR tag
%       %mapLims - corners of a square map [xmin,ymin,xmax,ymax}
%       
% 
%   OUTPUTS
%       w   weight of the state - a number between 0,1
% 
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #4
%   REINHARDT, BENJAMIN
%check if X is on the map - if it isn't, it gets 0 weight
if (X(1)<mapLims(1)|| X(2) < mapLims(2) || X(1) >mapLims(3)||X(2)>mapLims(4))
    w = 0;
    return
end
Q_good = [];
expectMeas = feval(h,X,ARs,sonars);
for i = 1:length(sonars)
    Q_good = blkdiag(Q_good,Q_sonar);
end
for j = 1:length(ARs)
    Q_good = blkdiag(Q_good,Q_AR);
end



%make sure everything is a row vector for mvnpdf
if size(expectMeas,1) > 1
    expectMeas = expectMeas';
end
if size(z,1) > 1
    z = z';
end
if size(z,2) ~= size(expectMeas,2)
    warning('uh oh');
end

 w = mvnpdf(z,expectMeas,Q_good);

end