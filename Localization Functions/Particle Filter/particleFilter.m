function [X_out,w_out] = particleFilter(X_in,z,u,p_u,p_z,stateNoise,resampler)
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
N = size(X_in,1); %size of state space
X_pred = zeros(size(X_in));
w = zeros(1,size(X_in,2));
w_out = zeros(1,size(X_in,2));
noise = feval(stateNoise);


for m=1:M
    X_pred(:,m) = feval(p_u,X_in(:,m),u)+noise(:,m)  ; %propigate each particle by updating according to the
    %dynamics and adding process noise
    X_pred(3) = mod(X_pred(3),2*pi);
    try
    w(m) = feval(p_z,X_pred(:,m),z); %find the weight of each propigated particle
    catch err
        disp('w not assigned?')
    end
    %if all the sensors can't be compared, give all the particles equal
    %weight
    
end
if max(w) == 0 
        w = ones(1,length(w))/length(w);
end
w = w/sum(w);

% %Resample step
%try

    X_out = feval(resampler,X_pred,w);
    for m = 1:M
    w_out(m) = feval(p_z,X_out(:,m),z); %find the weight of each resampled particle
    end
    if max(w_out) == 0 
        w_out = ones(1,length(w))/length(w);
    end
    w_out = w_out/sum(w_out);
    

    %pick new particles based on the weights
% catch err
%     global w_err
%     w_err = w;
%     print('w threw an error');
% end

% 


end