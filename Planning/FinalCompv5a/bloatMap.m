function [ newVertices ] = bloatMap(verticies,buffer)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here

% buffer = .1;
% 
global isLab
% figure(3)
% hold on
newVertices = [];
buffer1 = buffer;
for i = 1:size(verticies,1)
    if ((i < 5) && (isLab == 1))
        buffer = .01;
    else
        buffer = buffer1;
    end
    x1 = verticies(i,1);
    y1 = verticies(i,2);
    x2 = verticies(i,3);
    y2 = verticies(i,4);
    
    x = [x1 x2];
    y = [y1 y2];
%     plot(x,y,'b-');
%     hold on
    
    theta = atan2((y1 - y2),(x1 - x2));
    
    xOffset = -buffer*sin(theta);
    yOffset = buffer*cos(theta);
    xOffset1 = buffer*cos(theta);
    yOffset1 = buffer*sin(theta);
    
    x1New = x1 - xOffset + xOffset1;
    y1New = y1 - yOffset + yOffset1;
    
    x2New = x2 - xOffset - xOffset1;
    y2New = y2 - yOffset - yOffset1;
    
%     x = [x1New x2New];
%     y = [y1New y2New];
%     plot(x,y,'r-');
    
    newVertices = [newVertices; x1New y1New x2New y2New NaN NaN NaN NaN];
    
    x1New = x1 + xOffset + xOffset1;
    y1New = y1 + yOffset + yOffset1;
    
    x2New = x2 + xOffset - xOffset1;
    y2New = y2 + yOffset - yOffset1;
    
%     x = [x1New x2New];
%     y = [y1New y2New];
%     plot(x,y,'r-');
    
    newVertices(end,5:8) = [x2New y2New x1New y1New];% [x1New y1New x2New y2New];
    
end
%     
%     hold off
%     figure
    
end

