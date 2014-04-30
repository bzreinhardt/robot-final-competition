function X_est = genGuess(X_in, varargin)
%guess position based on a particle set or a mean
if nargin == 1
    X_est = X_in;
elseif nargin == 2
    w_in = varargin{1};
    X_weightmean = sum((ones(3,1)*w_in).*X_in,2);
    [biggestW, biggestI] = max(w_in);
    %assign best X
    X_best = X_in(:,biggestI);

    X_est = X_weightmean;

end
end