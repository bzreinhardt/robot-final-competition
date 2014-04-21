function [ pdf0,X,Y ] = makeGridMap( map, gridSize )
%makeGridMap creates a gridded map from a set of wall coordinates
%
%   INPUTS
%       map Nx4 matrix of wall endpoints (x1,y1,x2,y2)
%       gridSize	Number of rows x Number of columns  	2x1 [n m]
%
%   OUTPUTS
%       pdf0 - n x m matrix of initial pdf distribution - zero where there
%       are walls, equal everywhere else.
%       X - nxm matrix of x values at the left of each cell
%       Y - nxm matrix of y values at the bottom of each cell

%the min and max x,y values define the edges of the map
n = gridSize(1); m = gridSize(2);
xMax = max([map(:,1);map(:,3)]);
xMin = min([map(:,1);map(:,3)]);
yMax = max([map(:,2);map(:,4)]);
yMin = min([map(:,2);map(:,4)]);

if xMin == xMax
    xMax = xMin + 1;
end
if yMin == yMax
    yMin = yMin + 1;
end
x_coords = linspace(xMin, xMax, m+1);
y_coords = linspace(yMax, yMin ,n+1);
[X,Y] = meshgrid(x_coords(1,1:m),y_coords(2:n+1));
dx = X(1,2) - X(1,1); dy = Y(end-1,1) - Y(end,1);

% X = [X(1,:), X(1,end)+dx; X, (X(:,end)+dx)];
% Y = [Y(1,:)+dy, Y(1,end)+dy;Y, Y(:,end)];

%find the x and y coordinates of the LOWER edge of each cell

% --------
% |  |  |
% -------- Y(2)
% |  |  |
% -------- Y(1)
%

%X = X(1:n,1:m); Y = Y(2:n+1,2:m+1);

pdf0 = ones(size(X));
%go through each point and check whether there is a wall there
% for i = 1:m %step in x direction
%     for j = n:-1:1 %step in y direction
%         for k = 1:length(map(:,1)) %step through wall segments
%             
%             %if all those conditions are met, there is a wall in that cell
%             % set pdf of all cells with walls = 0
%             if wallInBox([X(j,i);Y(j,i)],dx,dy,map(k,:))
%                 pdf0(j,i) = 0;
%             end
%                
%         end
%     end
% end
%normalize across all the other squares
pdf0 = pdf0/sum(sum(pdf0));
end

%Bresenham's algorithm?


function isWall = wallInBox(lowerLeft, dx, dy, wall)
%finds whether the specified wall intersects teh specified box. If a wall
%is on the left or lower part of the box, it is in the box
%inputs (lowerleft) = [x;y] 

%if the wall exists in the cell, it will intersect one of the walls of the
%cell or be completely contained by the cell

%check if wall is contained by the cell
upperRight = lowerLeft + [dx;dy];
isWall = 0;
if wall(1) > lowerLeft(1) && wall(2) > lowerLeft(2) && wall(3) < upperRight(1) && wall(4) < upperRight(2)
    isWall = 1;
else
    %check all the edges of the wall
    %left
    [isectL,xL,yL,uaL]= intersectPoint(lowerLeft(1),lowerLeft(2),lowerLeft(1),upperRight(2),wall(1),wall(2),wall(3),wall(4));
    %right
     [isectR,xR,yR,uaR]= intersectPoint(upperRight(1),lowerLeft(2),upperRight(1),upperRight(2),wall(1),wall(2),wall(3),wall(4));
    %bottom
     [isectB,xB,yB,uaB]= intersectPoint(lowerLeft(1),lowerLeft(2),upperRight(1),lowerLeft(2),wall(1),wall(2),wall(3),wall(4));
    %top
     [isectT,xT,yT,uaT]= intersectPoint(lowerLeft(1),upperRight(2),upperRight(1),upperRight(2),wall(1),wall(2),wall(3),wall(4));
    %check it is along the right or top border, if it is remove it
   
     if isectL 
        if yL ~= upperRight(2)
            isWall = 1;
        end
    elseif isectR
         if yR ~= upperRight(2)
            isWall = 1;
        end
    elseif isectB
        if xB ~= upperRight(1)
            isWall = 1;
        end
    elseif isectT
         if xT ~= upperRight(1)
            isWall = 1;
        end
    end
    
    
end

end

