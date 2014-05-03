function [mapPlot] = plotMap( map )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

mx1 = map(:,1);
my1 = map(:,2);
mx2 = map(:,3);
my2 = map(:,4);
xVector = [];
yVector = [];
for i = 1:size(map,1)
    xVec = linspace(mx1(i),mx2(i));
    yVec = linspace(my1(i),my2(i));
    xVector = [xVector xVec];
    yVector = [yVector yVec];   
end
 mapPlot = plot(xVector,yVector,'r.');
end

