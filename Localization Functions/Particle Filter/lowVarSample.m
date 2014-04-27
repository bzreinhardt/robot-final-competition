function X_out = lowVarSample(X_in,w_in)
%random resampling for PF
if size(w_in,2) ~= size(X_in,2)
    error('number of weights does not match number of samples');
end
M = size(w_in,2);
X_out = zeros(size(X_in));
r = 1/M*rand(1);
c = w_in(1);
i = 1;
for m = 1:M
    U = r + (m-1)*1/M;
    while U > c
        i = i + 1;
        c = c+w_in(i);
    end
    X_out(:,m) = X_in(:,i);
    
end
end