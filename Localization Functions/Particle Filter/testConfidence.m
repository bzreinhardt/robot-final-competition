function [X_est,errCode] = testConfidence(X_in,w_in,measurements,predMeas,sonars,ARs)
%localize from a particle set
%
%   INPUTS
%       X_in - original set of particles 3xM (M is number of particles)
%       w_in - weights associated with input particles (1xM)
%
%
%   OUTPUTS
%       X_est - estimated X
%       confident - boolean of whether you're confident in the position

%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots

%constant that controls how much larger you need the best particle to be in
%order to be confident
margin = 0.05;
%threshold summed sonar error to trigger optional wall detection
sonarMargin = 0.1;

X_weightmean = sum((ones(3,1)*w_in).*X_in,2);
[biggestW, biggestI] = max(w_in);
%assign best X
X_best = X_in(:,biggestI);

X_est = X_weightmean;


%default err = 0;
errCode = 0;

%check beacon error
if ~isempty(ARs)
    %check to make sure position makes sense with sonars
    measAR = measurements(length(sonars)+1:length(sonars)+2);
    
    try
    predAR = predMeas(length(sonars)+1:length(sonars)+2);
    catch err
        disp(pred);
    end
    %if the norm of the AR error is large, throw an error
    if norm(predAR - measAR) > 0.5
        errCode = 1
        return;
    end
end

%check for systematic sonar error
sonarErr = predMeas(1:length(sonars))-measurements(1:length(sonars));
if sum(sonarErr) > sonarMargin
    errCode = 2
end




end