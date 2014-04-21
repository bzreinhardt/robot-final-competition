function [uN,uE,uS,uW] = findExpectDist(x,y,walls)
%finds the expected sonar distances based on a given grid and wall
%configuration
% INPUTS
%    x - x coordinates of midpoints of the grid point (1x1)
%     y - y coordinates of the midpoints of the gridpoint (1x1)
%     walls - Kx4 matrix of endpoints of the wall (x0,y0,x1,y1)
% OUTPUTS
%     uN - expected sonar measurement in the positive Y direction
%     uE - epxected sonar measurement in the positive X direction
%     uS - expected sonar measurement in the negative Y direction
%     uW - epxected sonar measurement in the negative X direction

%create an 'infinite' line in each direction
%initialize distance at 3
%check the line against each wall - if the intersection distance is the
%smallest, update distance
inf = 4; %distance beyond sonar range
max = 3;
xN = x;         yN = y + inf;
xE = x + inf;   yE = y;
xS = x;         yS = y - inf;
xW = x - inf;   yW = y;
uN = max; uE = max; uS = max; uW = max;
for k = 1:length(walls(:,1)) %check all the walls
    %check for intersections
    [isectN,xNorth,yNorth,ua]= intersectPoint(x,y,xN,yN,...
        walls(k,1),walls(k,2),walls(k,3),walls(k,4));
    [isectE,xEast,yEast,ua]= intersectPoint(x,y,xE,yE,...
        walls(k,1),walls(k,2),walls(k,3),walls(k,4));
     [isectS,xSouth,ySouth,ua]= intersectPoint(x,y,xS,yS,...
        walls(k,1),walls(k,2),walls(k,3),walls(k,4));
     [isectW,xWest,yWest,ua]= intersectPoint(x,y,xW,yW,...
        walls(k,1),walls(k,2),walls(k,3),walls(k,4));
    %check north
    if isectN && abs(y-yNorth) < uN
        uN = abs(y-yNorth);
    end
    %check east
   if isectE && abs(x-xEast) < uE
        uE = abs(x-xEast);
   end
   %check south
    if isectS && abs(y-ySouth) < uS
        uS = abs(y-ySouth);
    end
    %check west
     if isectW && abs(x-xWest) < uW
        uW = abs(x-xWest);
    end
   
end

end