function [X_mean,X_best,confident,variance] = localize(X_in,w_in)
%localize from a particle set
%
%   INPUTS
%       X_in - original set of particles 3xM (M is number of particles)
%       w_in - weights associated with input particles (1xM)
%
%
%   OUTPUTS
%       X_mean - mean particle position
%       confident - boolean of whether you're confident in the position

%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots

%constant that controls how much larger you need the best particle to be in
%order to be confident
margin = 0.05;

X_mean = mean(X_in,2);
[biggestW, biggestI] = max(w_in);
%assign best X
X_best = X_in(:,biggestI);
w = w_in;
w(biggestI) = -1;
[secondBiggestW, secondI] = max(w);
variance = var((X_in - X_mean*ones(1,size(X_in,2)))');
meanVar = mean(variance);
if meanVar < margin
confident = 1;
else
    confident = 0;
end

end