function plotKalmanPdf(mu,sigma, walls)
%plotPdf makes a plot of the probability distribution of a gaussian on a
%map
%  Inputs - mu: 2x1 vector of mean position
%           sigma - 2x2 covariance matrix of the position estimate 
%           walls: where the walls are located on the map
% outputs - none, makes a plot


%draw the position
plot(mu(1),mu(2),'x');
%draw the covariance 
e = plotCovEllipse(mu,sigma);
%draw the walls
for k = 1:length(walls(:,1))
    line([walls(k,1) walls(k,3)],[walls(k,2) walls(k,4)],'Color','r');
end

%contour(Xmid,Ymid,pdf);
%draw the grid
%draw the walls



end
