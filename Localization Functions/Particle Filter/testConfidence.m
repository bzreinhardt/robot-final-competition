function [X_est,err] = testConfidence(X_in,w_in,measurements,h,sonars,ARs)
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

X_weightmean = mean((ones(3,1)*w_in).*X_in,2);
[biggestW, biggestI] = max(w_in);
%assign best X
X_best = X_in(:,biggestI);
% w = w_in;
% w(biggestI) = -1;
% [secondBiggestW, secondI] = max(w);
%variance = var((X_in - X_mean*ones(1,size(X_in,2)))');
%meanVar = mean(variance);

X_est = mean(X_in,2);
%default err = 0;
err = 0;
%error if variance is too high
%error if beacon estimate is way off
if ~isempty(ARs)
    %check to make sure position makes sense with sonars
    measAR = measurements(length(sonars)+1:length(sonars)+2);
    pred = h(X_est,ARs,sonars);
    try
    predAR = pred(length(sonars)+1:length(sonars)+2);
    catch err
        disp(pred);
    end
    %if the norm of the AR error is large, throw an error
    if norm(predAR - measAR) > 0.5
        err = 1
        return;
    end
end


end