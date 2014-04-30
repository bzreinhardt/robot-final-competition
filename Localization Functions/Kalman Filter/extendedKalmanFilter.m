function [mu, sigma] = extendedKalmanFilter(mu0,sigma0,z, u,g,h, G, H, Q, R)
%extendedKalmanFilter : perform one complete EKF prediction and update step 
% 
%   [MU,SIGMA] = extendedKalmanFilter(mu0,sigma0, u, G, H, Q, R) returns
%   the updated belief [mu, sigma] that takes into account the original
%   belief mu0,sigma0, the measurements z, the control and measurement
%   covarience matricies Q and R, and the functions to linearize the
%   control and measurement dynamics, G and H and the control and
%   measurement dynamics, g and h
% 
%   INPUTS
%       mu0 - original state mean - 3x1
%       sigma0 - original state covariance matrix - 3x3
%       u - control input Kx1 where K is the number of inputs
%       z - measurement Mx1 where M is the number of sensors
%       g - nonlinear function between previous state/control and new state
%           function handle that takes state 
%       h - nonlinear function between state and expected measurement
%           function handle that takes state
%       G - function handle that calculates the linear map between x(t-1)
%           and x_bar(t) where x is the state. Assume all parameters except
%           the state and the control have been stuck in the function handle 
%       H - function handle that calculates the linear map between x(t-1)
%           and the expected measurement. Assume all parameters except
%           the state have been stuck in the function handle
%       Q - measurement covariance matrix MxM where M is the number of
%           sensors
%       R - control covariance matrix, here 2x2 for the odometry
%  
%
%   OUTPUTS
%       mu  - new state mean - 3x1     
%       sigma - new state covariance matrix 3x3

% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework 4
%   Reinhardt, Benjamin

%find jacobians
G_eval = feval(G,mu0,u);
H_eval = feval(H,mu0);
%prediction step
mu_bar = feval(g,mu0,u); %predict mu
sigma_bar = G_eval * sigma0 * G_eval' + R; %predict sigma
%update step

kt = sigma_bar*H_eval'*(H_eval*sigma_bar*H_eval'+Q)^(-1); %kalman gain
mu = mu_bar + kt*(z - feval(h,mu_bar))';
sigma = (eye(size(sigma_bar)) - kt*H_eval)*sigma_bar;
dbstop if warning


end