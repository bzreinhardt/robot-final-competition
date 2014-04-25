function particles = initializePF(waypoints,angles)
%INITIALIZEPF - create an initial set of state particles from possible
%starting positions
%%%%
% INPUTS
% waypoints - set of x,y coordinates where the robot could possibly start -
% IN THE FORMAT THAT THEY'VE BEEN GIVEN - [Nx2]
%%%
% OUTPUTS
% particles - set of [x;y;theta] particles. 4 particles/ waypoint, at
% 0,pi/2,pi,3pi/2 - [3xN]
particles = [];
for i = 1:size(waypoints,1)
    newParticles = [ones(1,size(angles,2))*waypoints(i,1);ones(1,size(angles,2))*waypoints(i,2);angles];
    particles = [particles,newParticles];
end

end