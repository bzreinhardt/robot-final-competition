function [newVisited,newUnvisited,alert] = checkWayPts(X,wayPtsVisited,wayPtsUnvisited,closeEnough)
%CHECKWAYPTS - checks whether the robot has reached a waypoint
%INPUTS
% X - robot pose [3x1]
% wayPtsVisited - x,y coordinates of visted waypts [Nx2]
% wayPtsUnvisited - x,y coordinates of unvisteded waypts [Mx2]
% closeEnough - distance from waypt considered close enough to have visited
% it (scalar)
% 
%OUTPUTS
% newVisited - updated x,y coordinates of visited waypts 
% newUnvisted - updated x,y coordinates of unvisted waypts
% alert - 0 if no waypt visited 1 if waypt visited

dist = wayPtsUnvisited - ones(size(wayPtsUnvisited,1),1)*X(1:2)';
normDist = sqrt(sum(abs(dist).^2,2));
[closeDist,closeI] = min(normDist);
if closeDist < closeEnough
    %If you're next to a waypt
    newVisited = [wayPtsVisited;wayPtsUnvisited(closeI,:)];
    ind = 1:size(wayPtsUnvisited,1);
    newInd = ind(ind~=closeI);
    newUnvisited = wayPtsUnvisited(newInd,:);
    alert = 1;
else
    %if you're not next to the waypt keep everything the same
    newVisited = wayPtsVisited;
    newUnvisited = wayPtsUnvisited;
    alert = 0;
end
end