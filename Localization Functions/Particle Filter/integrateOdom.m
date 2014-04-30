function [ X_f ] = integrateOdom( X_0, varargin )
% INTEGRATEODOM: Calculate final global pose from odometer data
% 
%   [ X_f ] = integrateOdom( X_0, d, phi ) returns the 
%   pose of the robot after one time step calculated by dead reconing from
%   the initial pose, the distance travelled and the angle turned
% 
%   INPUTS
%       X_0      initial pose in global coordiantes - [x, y, theta]' 3x1
%       varargin: either
%       d        distance travelled  1xN
%       phi       angle turned in degrees 1xN
%        or
%       [d phi]' 2xN 
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

if nargin == 2
    u = varargin{1};
    d = u(1,:);
    phi = u(2,:);
elseif nargin == 3
    d = varargin{1};
    phi = varargin{2};
else
    error('wrong number of arguments');
end

%assume that the wheels both turn at the same rate during the time step.
%This causes the robot to drive along an arc with radius r
%initial case
if phi(1) ~= 0
    r = d(1)/(phi(1));
    dX_robot = [r*sin(phi(1)); r-r*cos(phi(1))]; %change in position robot frame
elseif phi(1) == 0
    dX_robot = [d(1);0];
end


global_Q_robot = [cos(X_0(3,1)) -sin(X_0(3,1)); sin(X_0(3,1)) cos(X_0(3,1))]; %rotation matrix from robot frame to global frame
dX_global = global_Q_robot*dX_robot; %change in position global frame
X_global = dX_global+X_0(1:2,1); %new absolute position in global frame
theta_new = mod(X_0(3,1)+phi(1),2*pi); %new heading - keep heading between 0 and 2pi
X_f(:,1) = [X_global;theta_new]; %new pose

if length(d) > 1
    for i = 2:length(d)
        X_f(:,i) = integrateOdom(X_f(:,i-1),d(i),phi(i));
    end
end


end

