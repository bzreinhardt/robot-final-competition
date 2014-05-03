function [ dist ] = findDist(X1,X2)
%FINDDIST finds the euclidean distance between an array of vectors X1 and a
%single vector of the same size, X2

dist = sqrt(sum((X1-X2*ones(1,size(X1,2))).^2,1));


end

