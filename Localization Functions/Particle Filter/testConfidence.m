function [locEvent,predictMeasGuess] = testConfidence(X,measurements,h,sonars,ARs)
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
errThreshold = 0.1;
%threshold summed sonar error to trigger optional wall detection
sonarMargin = 0.1;
predictMeasGuess = feval(h,X,ARs,sonars);
%difference of measurements at guessed position from true measurements (ie
%if the guess predicted measurement is too high, err will be positive
measErr = sum(predictMeasGuess-measurements);

%default err = 0;
locEvent = 0;

%error too high
if abs(measErr) > errThreshold
    locEvent = 1;
    return;
end
%

% %check beacon error
% if ~isempty(ARs)
%     %check to make sure position makes sense with sonars
%     measAR = measurements(length(sonars)+1:length(sonars)+2);
%     
%     try
%     predAR = predMeas(length(sonars)+1:length(sonars)+2);
%     catch err
%         disp(pred);
%     end
%     %if the norm of the AR error is large, throw an error
%     if norm(predAR - measAR) > 0.5
%         errCode = 1
%         return;
%     end
% end

%check for systematic sonar error
sonarErr = predictMeasGuess(1:length(sonars))-measurements(1:length(sonars));
if sum(sonarErr) > sonarMargin
    locEvent = 2;
end




end