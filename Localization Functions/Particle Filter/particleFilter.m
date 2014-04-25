function [X_out,w_out] = particleFilter(X_in,z,u,p_u,p_z,R)
%particleFilter : perform one complete PF prediction and update step
%
%   [X_OUT] = particleFilter(X_in,z, g,h, p_u,p_z) returns
%   the updated particle set X_out that takes into account the original
%   particle set X_in, the measurements z, and the control and
%   measurement dynamics, g and h, and the functions p_u =
%   x_t| x_t-1,u  and p_z = p(z_t|x_t)
%
%   INPUTS
%       X_in - original set of particles 3xM (K is number of particles)
%       z - measurement 1xK where K is the number of sensors
%       u - control input
%      p_u - function that updates a single state with probalistic noise
%           and dynamics
%       p_z - function that finds the probability of the actual measurement
%           given a certain state
%
%
%   OUTPUTS
%       X_out - new set of particles 3xM (M is number of particles)
%       w_out - determined weights of the X_in particles - 1xM

%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework 4
%   Reinhardt, Benjamin

%prediction step
M = size(X_in,2); %number of particles

for m=1:M
    X_pred(:,m) = feval(p_u,X_in(:,m),u) + R*randn(size(X_in,1),1); %propigate each particle by updating according to the
    %dynamics and adding process noise
    w(m) = feval(p_z,X_pred(:,m),z); %find the weight of each propigated particle
    %if all the sensors can't be compared, give all the particles equal
    %weight
    
end
if max(w) == 0 
        w = ones(1,length(w))/length(w);
    end
% %Resample step
try
y = randsample(M,M,true,w); %pick new particles based on the weights
catch err
    global w_err
    w_err = w;
    print('w threw an error');
end
for m = 1:M
    X_out(:,m) = X_pred(:,y(m));
    w_out(m) = feval(p_z,X_out(:,m),z); %find the weight of each resampled particle
end
% 


end