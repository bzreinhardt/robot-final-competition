function[G_t] = GjacDiffDrive(x,u)
% GjacDiffDrive : output the jacobian G_t = dg/dx(t-1) for a planar robot
% using odometry as the input, u
% 
%   G_t = GJACDIFFDRIVE(x) returns 
%   the odometry information jacobian for the pose, x(t-1). 
% 
%   INPUTS
%       x - 3x1 vector of [x;y;theta] pose
%       u - 2x1 vector of odometry data - [d,theta]
% 
%   OUTPUTS
%       G_t       3x3 jacobian matrix evaluated at x and u
% 

% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework 4
%   Reinhardt, Benjamin

%see homework writeup for the actual differentiation math
dh_dx = [1;0;0];
dh_dy = [0;1;0];
d = u(1); phi = u(2); theta = x(3);
if phi ~= 0

r = d/phi;


dh_dTheta = zeros(3,1);
dh_dTheta(1:2,1) = [-sin(theta) -cos(theta);cos(theta) -sin(theta)]*...
    [r*sin(phi);r-r*cos(phi)];
elseif phi == 0
    dh_dTheta = [0;0;1];
end
dh_dTheta(3) = 1;

G_t = [dh_dx,dh_dy,dh_dTheta];

end