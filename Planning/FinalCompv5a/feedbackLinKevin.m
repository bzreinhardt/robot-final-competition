function[Vel,W] = feedbackLin(velX,velY, theta, eps)
% Outputs a forward and angular velocity given x and y velocities
% angle theta and epsilon.
%
%
%
%
%

epsArray = [1 0; 0 (1/eps)];
RBI = [cos(theta) sin(theta); -sin(theta) cos(theta)];
velArray = [velX; velY];
Output = epsArray*RBI*velArray;
Vel = Output(1);
W = Output(2);