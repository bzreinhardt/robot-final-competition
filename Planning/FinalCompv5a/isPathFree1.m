function [ freePath ] = isPathFree1( map,x1,y1,x2,y2 )
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
% returns 1 if path is free, 0 if path is obstructed

% hold on

inObstacle = 0;
freePath = 0;

lines1 = map(:,1:4);
lines2 = map(:,5:8);
map = [lines1; lines2];

numLines = size(map,1);

for i = 1:numLines
    x3 = map(i,1);
    y3 = map(i,2);
    x4 = map(i,3);
    y4 = map(i,4);
    
    [isect,x,y,ua]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
    if (isect == true)
        inObstacle = 1;
        freePath = 0;
        return
    end
end    




if (inObstacle == 1)
    freePath = 0;
else
    freePath = 1;
end

end