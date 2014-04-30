function [mu,sigma] = EKF(mu0,sigma0,z, u,g,h, G, H, Q_sonar,Q_AR, R,sonars,ARs)
% wrapper function for the extended kalman filter

%build the covariance matrix of the right size
Q_good = [];

for i = 1:length(sonars)
    Q_good = blkdiag(Q_good,Q_sonar);
end
for j = 1:length(ARs)
    Q_good = blkdiag(Q_good,Q_AR);
end

h_ekf = @(X)h(X,ARs,sonars);
H_ekf = @(X)H(X,h,ARs,sonars);
[mu, sigma] = extendedKalmanFilter(mu0,sigma0,z, u,g,h_ekf, G, H_ekf, Q_good, R);

end