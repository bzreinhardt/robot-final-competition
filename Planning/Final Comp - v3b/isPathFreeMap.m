function [ freePath ] = isPathFreeMap( map,buffer,x1,y1,x2,y2 )
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
% returns 1 if path is free, 0 if path is obstructed

hold on

inObstacle = 0;
freePath = 0;

lines1 = map(:,1:4);
lines2 = map(:,5:8);
map = [lines1; lines2];

numLines = size(map,1);

    theta = atan2((y1 - y2),(x1 - x2));
    
    xOffset = -buffer*sin(theta);
    yOffset = buffer*cos(theta);
    xOffset1 = buffer*cos(theta);
    yOffset1 = buffer*sin(theta);
    
    x2New = x2 - xOffset1;
    y2New = y2 - yOffset1;

%     x = [x1 x2New];
%     y = [y1 y2New];
%     plot(x,y,'b-');

for i = 1:numLines
    x3 = map(i,1);
    y3 = map(i,2);
    x4 = map(i,3);
    y4 = map(i,4);
    
    [isect,x,y,ua]= intersectPoint(x1,y1,x2New,y2New,x3,y3,x4,y4);
    if (isect == true)
        inObstacle = 1;
        freePath = 0;
        return
    end
end    

%     theta = atan2((y1 - y2),(x1 - x2));
%     
%     xOffset = -buffer*sin(theta);
%     yOffset = buffer*cos(theta);
%     xOffset1 = buffer*cos(theta);
%     yOffset1 = buffer*sin(theta);
    
    
    x1New = x1 - xOffset;
    y1New = y1 - yOffset;
    
    x2New = x2 - xOffset - xOffset1;
    y2New = y2 - yOffset - yOffset1;
    
%     x = [x1New x2New];
%     y = [y1New y2New];
%     plot(x,y,'b-');
    
%     %%%%%%
%     x1 = x1 + 2*xOffset;
%     y1 = y1 + 2*yOffset;
%     x2 = x2 + 2*xOffset; % - 2*xOffset1;
%     y2 = y2 + 2*yOffset; % - 2*xOffset1;
%     
%     x = [x1 x2];
%     y = [y1 y2];
%     plot(x,y,'b-');
%     %%%%%%
for i = 1:numLines
    x3 = map(i,1);
    y3 = map(i,2);
    x4 = map(i,3);
    y4 = map(i,4);
    
    [isect,x,y,ua]= intersectPoint(x1New,y1New,x2New,y2New,x3,y3,x4,y4);
    if (isect == true)
        inObstacle = 1;
        freePath = 0;
        return
    end
end    

    x1New = x1 + xOffset;
    y1New = y1 + yOffset;
    x2New = x2 + xOffset - xOffset1;
    y2New = y2 + yOffset - yOffset1;
    
%     x = [x1New x2New];
%     y = [y1New y2New];
%     plot(x,y,'b-');
    
for i = 1:numLines
    x3 = map(i,1);
    y3 = map(i,2);
    x4 = map(i,3);
    y4 = map(i,4);
    
    [isect,x,y,ua]= intersectPoint(x1New,y1New,x2New,y2New,x3,y3,x4,y4);
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

