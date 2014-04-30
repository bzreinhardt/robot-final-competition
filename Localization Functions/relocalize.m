function [relocParts] = sigmaRelocalize(X_last,sigma,M)
%relocalize generates a new particle set around a last known position

 R = 2*chol(sigma);
 relocParts = repmat(X_last,1,M) + R*randn(size(X_last,1),M);
end
