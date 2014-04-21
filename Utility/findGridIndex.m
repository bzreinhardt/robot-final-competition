function [index] = findGridIndex(pt, X_grid, Y_grid)
% findGridIndex finds the index on a grid of points
%INPUTS
% pt = matrix of x,y coordinates - Nx2 [x y; x y; etc]
% X_grid - evenly spaced grid coordinates in the X direction 1xM, Mx1, KxM,
%       X_grid values increase with increasing indicies
% Y_grid - evenly spaced grid coordinates in the Y direction 1xK, Kx1, KxM
%       Y_grid values decrease with increasing idicies
%OUTPUTS
% index - list of indicies of input points 2xN or Nx2 (N in direction of
% pt N) in row,column (so, [Yplace Xplace]

[m_pt, n_pt] = size(pt);
[m_x, n_x] = size(X_grid);
[m_y, n_y] = size(Y_grid);
%parse input matricies
if m_x ~=1 && n_x ~= 1
    X = X_grid(1,:);
elseif m_x > 1
    X = X_grid';
else 
    X = X_grid;
end

if m_y ~=1 && n_y ~= 1
    Y = Y_grid(:,1);
elseif n_y > 1
    Y = Y_grid';
else 
    Y = Y_grid;
end

index = zeros(m_pt,2);

for i = 1:m_pt
    [minDiffx, xminI] = min(abs(X-pt(i,1)));
    [minDiffy, yminI] = min(abs(Y-pt(i,2)));
    
    if pt(i,1)<X(xminI)
        xminI = xminI -1;
    end
    if pt(i,2)<Y(yminI)
        yminI = yminI+1;
    end
    
    index(i,:) = [yminI,xminI];
end

