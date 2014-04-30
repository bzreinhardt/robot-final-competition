function noise = normStateNoise(size_u,R)
%pfLocalizeR adds the random component to the input in aparticle filter
%Inputs
%R - process covariance matrix
%size_X - 2x1 or 1x2 matrix corresponding to the size of particles(first
%element) and number of particles (second element)
%outputs 
%r_out - nxm position vector to add to particles
n = size_u(1);
m = size_u(2);
noise = R*randn([n,m]);


end