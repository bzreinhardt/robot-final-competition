function[H_t] = HjacSonarAR(X,h,ARs,sonars)
%HjacSonarAR : output the jacobian H_t = dh/dx(t-1) for a planar robot
% using sonar and beacons
% 
%   H_t = GJACDIFFDRIVE(x) returns 
%   the measurement information jacobian for the pose, x(t-1). 
% 
%   INPUTS
%       X - 3x1 vector of [x;y;theta] pose
%       ARs - array of beacon numbers
%       map         N-by-4 matrix containing the coordinates of walls in 
%                   the environment: [x1, y1, x2, y2]
%       robotRad    robot radius (meters)


% 
%   OUTPUTS
%       H_t       (number of sonars + number of beacons*2)x3 jacobian matrix 
%      evaluated at x t
% 

% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Final Competition
%   Reinhardt, Benjamin

%see homework writeup for the actual differentiation math


%from the taylor expansion of the function, we can derive that f'(x) ~
%f(x+h) - f(x)/h for small h this becomes the definition of f'(x) as h -> 0
%
if max(size(X) ~= [3 1]) == 1
    warning('x is the wrong size');
end
  
%let h be small wrt the throw of the sonar
dx = 0.01; dy = dx; dtheta = 0.01;

predMeas = feval(h,X,ARs,sonars);
predMeas_dx = feval(h,X+[dx;0;0],ARs,sonars);
predMeas_dy = feval(h,X+[0;dy;0],ARs,sonars);
predMeas_dtheta = feval(h,X+[0;0;dtheta],ARs,sonars);
H_t = [(predMeas_dx-predMeas)/dx;(predMeas_dy-predMeas)/dy;...
    (predMeas_dtheta-predMeas)/dtheta]';
%     ranges_x = sonarPredict(X,map,robotRad,angles,maxRange);
%     ranges_xh = sonarPredict(X+[h_x;0;0],map,robotRad,angles,maxRange);
%     ranges_y = sonarPredict(X,map,robotRad,angles,maxRange);
%     ranges_yh = sonarPredict(X+[0;h_y;0],map,robotRad,angles,maxRange);
%     ranges_theta = sonarPredict(X,map,robotRad,angles,maxRange);
%     ranges_thetah = sonarPredict(X+[0;0;h_theta],map,robotRad,angles,maxRange);
%     dR_dx = (ranges_xh-ranges_x)/h_x;
%     dR_dy = (ranges_yh-ranges_y)/h_y;
%     dR_dtheta = (ranges_thetah-ranges_theta)/h_theta;
%    % H_t = [dR_dx,dR_dy,dR_dtheta];

end