function X_out = randSample(X_in,w_in)
%random resampling for PF
if size(w_in,2) ~= size(X_in,2)
    error('number of weights does not match number of samples');
end
X_out = zeros(size(X_in));
y = randsample(M,M,true,w);
for m = 1:size(w_in,2)
    
    X_out(:,m) = X_in(:,y(m));
    
end
end