function X_est = genGuess(X_in, varargin)
%guess position based on a particle set or a mean
if nargin == 1
    X_est = X_in;
elseif nargin == 2
    w_in = varargin{1};
    X_xyMean = mean(X_in(1:2,:),2);
    X_thetaMean = atan2(mean(sin(X_in(3,:)),2),mean(cos(X_in(3,:)),2));
    X_mean = [X_xyMean;X_thetaMean];
    [biggestW, biggestI] = max(w_in);
    %assign best X
    X_best = X_in(:,biggestI);
    
    X_est = X_mean;
elseif nargin == 3
    w_in = varargin{1};
    if varargin{2} == 0
        %if you haven't localized for the first time use the best particle
        [biggestW, biggestI] = max(w_in);
        X_best = X_in(:,biggestI);
        X_est = X_best;
    else
        %if you have localized for the first time (1 hypothesis) use the
        %mean particle
        X_xyMean = mean(X_in(1:2,:),2);
        X_thetaMean = atan2(mean(sin(X_in(3,:)),2),mean(cos(X_in(3,:)),2));
        X_mean = [X_xyMean;X_thetaMean];
        X_est = X_mean;
    end
end