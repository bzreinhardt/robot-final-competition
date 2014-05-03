function [ freePath ] = isPathFree( polyArray,xArray,yArray )
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
% returns 1 if path is free, 0 if path is obstructed

freePath = 0;
sizeXArray = size(xArray,2);
for i = 1:sizeXArray
    x = xArray(i);
    y = yArray(i);
    inObstacle = pointInPoly(polyArray,x,y);
    if (inObstacle == 1)
        break
    end
end

if (inObstacle == 1)
    freePath = 0;
else
    freePath = 1;
end

end

