function plotPdf(X,Y,pdf, walls)
%plotPdf makes a surface plot of the probability distribution of a pdf
%  Inputs - X : nxm matrix of lower left x coordinates
%           Y : nxm matrix of lower left y coordinates
%           pdf: nxm matrix of probabilities for each cell
% outputs - none, makes a plot
dx = abs(X(1,2) - X(1,1)); dy = abs(Y(end-1,1) - Y(end,1));
[Xmid,Ymid] = calcCenters(X,Y);
[n,m] = size(X);


%draw the grid
for i = 1:n
    for j = 1:m
        h = rectangle('Position',[X(i,j),Y(i,j),dx,dy]);
        if pdf(i,j) == 0
            set(h,'FaceColor','k');
        else 
            %got the following code from stackexchange
            f = pdf(i,j); % your float
            cm = colormap; % returns the current color map
            colorID = max(1, sum(f > [0:1/length(cm(:,1)):1])); 
            myColor = cm(colorID, :); % returns your color
            set(h,'FaceColor',myColor);
        end
        
    end
end
for k = 1:length(walls(:,1))
    line([walls(k,1) walls(k,3)],[walls(k,2) walls(k,4)],'Color','r');
end

%contour(Xmid,Ymid,pdf);
%draw the grid
%draw the walls



end

function [Xmid,Ymid] = calcCenters(X,Y)
%calcCenters creates a grid of centerpoints for a grid of lowerleft points
% Inputs, X, Y - nxm matricies of lower left X and Y coordinates
% outputs Xmid, Ymid nxm matricies of middle X and Y coordinates
dx = abs(X(1,2) - X(1,1)); dy = abs(Y(end-1,1) - Y(end,1));
Xmid = X + dx/2; Ymid = Y + dy/2;
end
