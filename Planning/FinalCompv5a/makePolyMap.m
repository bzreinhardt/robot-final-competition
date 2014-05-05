function [ output_args ] = makePolyMap( polyFile,xMin,xMax,yMin,yMax )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

xBound = [xMin xMax xMax xMin xMin];
yBound = [yMin yMin yMax yMax yMin];
figure(2)
plot(xBound,yBound,'r-');
hold on
for i = 1:size(polyFile,1)
   polyArray = drawPoly(polyFile,i);
   polyX = polyArray(1,:);
   polyX = polyX(1:polyX(end));
   polyY = polyArray(2,:);
   polyY = polyY(1:polyY(end));
   fill(polyX,polyY,'b');   
end
 hold off
end

