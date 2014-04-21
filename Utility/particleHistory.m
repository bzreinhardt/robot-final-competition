function [bestParticles, meanParticles] = particleHistory(particles)
%% Info
% particleHistory finds a series of best particles and average particles
% from a particle filter dataset

%INPUT
% particles (4*NxM+1 matrix - N time steps [w,x,y,z] for each of M
% particles, time in first column
%OUTPUT
% bestParticles - highest weighted particles at each time step 3xN
% meanParticles - average particle at each time step 3xN

bestParticles = []; meanParticles = [];
for i = 1:(length(particles(:,1))/4)-1
    [maxW, maxI] = max(particles(4*i+1,2:end));
    bestParticles = [bestParticles particles(i*4-2:i*4,maxI+1)];
    meanParticles = [ meanParticles mean(particles(i*4-2:i*4,2:end),2)];
end

end