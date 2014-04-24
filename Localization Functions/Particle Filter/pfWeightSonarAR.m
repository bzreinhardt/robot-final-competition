function [w] = pfWeight(X,nhz_sonar,h_sonar,z_AR,h_AR,Q_sonar,Q_AR)
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
%       Q - measurement covariance matrix KxK
%       
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
goodMeas =[];Q_good = [];
expectMeas = feval(h_sonar,X);

for i = 1:length(z_sonar)
    if ~isnan(z(i)) && ~isnan(expectMeas(i))
        goodMeas = [goodMeas, i];
        Q_good = [Q_good Q_sonar(i,i)];
    end
end
Q_good = diag(Q_good);
%make sure everything is a row vector for mvnpdf
if length(expectMeas(:,1)) > 1
    expectMeas = expectMeas';
end
if length(z(:,1)) > 1
    z = z';
end
if length(goodMeas) == 0
    %if the real measurements and the expected particle measurements do not
    %overlap give the particle a 3 sigma weight. DESIGN DECISION. I'm not
    %entirely sure how to hanle this
    mu = [0]; z = [3]; sigma = 1;
    w = mvnpdf(z,mu,sigma);
else
    mu = expectMeas(goodMeas);
    goodZ = z(goodMeas);
    w = mvnpdf(goodZ,mu,Q_good);
end


end