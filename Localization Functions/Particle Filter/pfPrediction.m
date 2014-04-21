function [X] = pfPrediction(X0,g,u,R)
% PFPREDICTION: Calculate a stochastic new particle state
% 
%   [ newState ] = pfPrediction(oldstate,g,u,R) takes an initial particle
%   state for a PF, applies the system dynamics and noise
% 
%   INPUTS
%       X0 - initial particle state
%       g  - noiseless dynamics function handle, takes (state,control) as
%           arguments
%       u  - applied control input
%       
%       
% 
%   OUTPUTS
%       X_f   final pose in global coordinates - [x,y,theta]' 3xN
% 
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
%   REINHARDT, BENJAMIN
end