function X_out = noResample(X_in,w_in)
%no resampling for PF
if size(w_in,2) ~= size(X_in,2)
    error('number of weights does not match number of samples');
end

X_out = X_in;
end
