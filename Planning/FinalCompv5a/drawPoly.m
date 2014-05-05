function [ polyArray ] = drawPoly( polyFile, index )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

numEntries = 16;
polyArray = [];
polyFile = polyFile(index,:);
j = 1;
i = 1;
k = 0;
    while (i < numEntries) % (polyFile(i) ~= 0)
        polyArray(1,j) = polyFile(i);
        polyArray(2,j) = polyFile(i + 1);
        if (polyFile(i + 1) ~= 0)
            k = k + 1;
        end
        j = j + 1;
        i = i + 2;
        
        if i >= size(polyFile)
            break
        end
    end
    k = [k;k];
    polyArray = [polyArray k];
end

