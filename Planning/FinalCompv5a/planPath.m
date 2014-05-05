function [ shortestPath pathDist relocalize ptIdx wpOrWall] = planPath(walls,optWalls,wpOrWall,currPose,unvisitedWPs,visitedWPs )
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here

% INPUTS:
%     walls - Nx4 
%               [ wall1X1 wall1y2 wall1X2 wall1Y2
%                 wall2X1 wall2Y2 wall2X2 wall2Y2
%                 wallNX1 wallNX2 wallNX3 wallNX4 ]
%     optWalls - Nx4
%               
%     wpOrWall - scalar
%               1 if we are looking for a path to a waypoint
%               2 if we are looking to bump an optional wall
%     currPose - 3x1 
%               [x;y;theta]
%     unvisitedWPs - 2xN 
%               (columns of [x;y] waypoints)
%     visitedWPs - 2xN 
%               (columns of [x;y] waypoints)
%     
% OUTPUTS:
%     shortestPath - Nx3 
%               [NodeIndex xcoord ycoord; etc]
%     pathDist - scalar 
%               path distance
%     relocalize - scalar
%               value of 1 if the starting position is in a wall
%               default value of 0


relocalize = 0;
ptIdx = NaN;
wallBuffer = .25;
robotRadius = .25;
% if (wpOrWall == 1)
%     allWalls = [walls; optWalls];
% elseif (wpOrWall == 2)
%     allWalls = walls;
% end
allWalls = walls;

[maxX maxY minX minY] = findMaxMin(allWalls); % findMaxMin(verticies);

% boundary = [0 0 0 100 100 100 100 0];
boundary = [minX minY minX maxY maxX maxY maxX minY];
% 
verticies = bloatMap(allWalls, .3); % bloatMap(allWalls,robotRadius); 

if ((currPose(1,1) < minX) || (currPose(1,1) > maxX) || (currPose(2,1) < minY) || (currPose(2,1) > maxY))
    shortestPath = NaN;
    pathDist = NaN;
    ptIdx = NaN;
    relocalize = 1;
    return
end

env = verticies;
border = boundary;
x = []; y = []; shape = []; constraints = [];
k = 1; shapeStart = 1;
for i = 1:size(verticies,1)
    shapeStart = k;
    % test if current position is in an obstacle
     polyXs = [verticies(i,1) verticies(i,3) verticies(i,5) verticies(i,7)];
     polyYs = [verticies(i,2) verticies(i,4) verticies(i,6) verticies(i,8)];
     [IN ON] = inpolygon(currPose(1,1),currPose(2,1),polyXs,polyYs); %
     if IN == 1
         shortestPath = NaN;
         pathDist = NaN;
         ptIdx = NaN;
         relocalize = 1;
         return
     end
        
    for j = 1:2:size(verticies,2)
        if (verticies(i,j) ~= 0 && verticies(i,j+1) ~= 0)
            x = [x;verticies(i,j)]; 
            y = [y;verticies(i,j+1)];
            shape = [shape;i];
            constraints = [constraints; k k+1];
            
            if j+2 > size(verticies,2) || (verticies(i,j+2) == 0 && verticies(i,j+3) == 0 && j>3)
                constraints(end,:) = [k shapeStart];
            end
            k = k + 1;
%         elseif (verticies(i,j-2) ~= 0 && verticies(i,j-1) ~= 0)
%             constraints(end,:) = [k shapeStart];
        end
        
    end
end
% figure
%constraints = [10 17];
%add the boundaries of the object
for i = 1:2:size(boundary,2)
    x = [x;boundary(i)];
    y = [y;boundary(i+1)];
    shape = [shape;0];
    
end
% x = [x;0; 0; 100; 100];
% y = [y;0; 100; 0; 100];
% shape = [shape; 0;0;0;0];
% do a delaunay decomposition of the workspace
global isLab
if (isLab == 1)
    DT = DelaunayTri(x,y,constraints); % DT = delaunayTriangulation(x,y,constraints);
else
    DT = delaunayTriangulation(x,y,constraints);
end
% figure(1);clf;triplot(DT); hold on
%% Find the triangles that are actually in shapes
% plot the actual obstacles    
% for i = 1:max(shape)
%     patch(x(shape == i),y(shape == i),'r');
%     hold on
% end
%find the delaunay edges that go through the shapes and mark them
E = edges(DT);
if (isLab == 1)
    nodes = DT.X;
else
    nodes = DT.Points;
end
%DO I NEED THE BELOW CODE?
% shapeEdges = [];
% startNode = 1;
% endNode = 1;
% for i = 1:max(shape)
% end

%find the midpoints of all the edges
    x1 = nodes(E(:,1),1);
    x2 = nodes(E(:,2),1);
    y1 = nodes(E(:,1),2);
    y2 = nodes(E(:,2),2);
    midX = abs(x1-x2)/2+min([x1,x2],[],2);
    midY = abs(y1-y2)/2+min([y1,y2],[],2);

% plot(midX,midY,'x'); hold on

polygons = verticies; % bloatMap(allWalls,wallBuffer); %  

inObs = [];
onObs = [];
for i = 1:max(shape) %size(polygons,1)
%     polyXs = [polygons(i,1) polygons(i,3)];
%     polyYs = [polygons(i,2) polygons(i,4)];
    [IN ON] = inpolygon(midX,midY,x(shape == i),y(shape == i)); % inpolygon(midX,midY,polyXs,polyYs); %
    inObs = [inObs,IN];
    onObs = [onObs,ON];
end
%outside if IN == 0 and ON == 0
%boundary if IN == 1 and ON == 1
%inside if IN == 1 and ON == 0
notEdge = ~onObs;
inAll = inObs.*notEdge;
interiorEdges = sum(inAll,2);
%DT.Constraints = E(interiorEdges == 1,:);
% inX = midX(interiorEdges == 1);
% inY = midY(interiorEdges == 1);
% plot(inX,inY,'gx');

graphE = E(~interiorEdges,:);
%adjMrxS = sparse([graphE(:,1);graphE(:,2)],[graphE(:,2);graphE(:,1)],ones(2*size(graphE,1),1),size(graphE,1),size(graphE,1));
adjMrxB = zeros(size(nodes,1),size(nodes,1));
for i = 1:size(graphE,1)
    adjMrxB(graphE(i,1),graphE(i,2)) = 1;
    adjMrxB(graphE(i,2),graphE(i,1)) = 1;
end
%plot the resulting thing
% figure(2);
% gplot(adjMrxB,nodes,'-o');
% for j = 1:size(nodes,1),
%     text(nodes(j,1),nodes(j,2),int2str(j),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','r');
% end
%%
%another way of figuring out what to get rid of - if the center of the
%triangle is in the same place as an object
inObs2 = [];
if (isLab == 1)
    centers = incenters(DT,[1:size(DT,1)]');
else
    centers = incenter(DT,[1:size(DT,1)]');
end
for i = 1:max(shape) %  size(polygons,1)% 
%     polyXs = [polygons(i,1) polygons(i,3)];
%     polyYs = [polygons(i,2) polygons(i,4)];    
    [IN ON] = inpolygon(centers(:,1),centers(:,2),x(shape == i),y(shape == i)); % polyXs,polyYs); %
    inObs2 = [inObs2,IN];
%     onObs = [onObs,ON];
end
interiorShapes = sum(inObs2,2);
%plot the 
% figure(3);clf;triplot(DT); hold on
% for i = 1:max(shape)
%     patch(x(shape == i),y(shape == i),'r');
%     hold on
% end
%plot(centers(:,1),centers(:,2),'x'); hold on
inShapes = centers(interiorShapes == 1,:);
outShapes = centers(interiorShapes == 0,:);
% plot(inShapes(:,1),inShapes(:,2),'xg');
% plot(outShapes(:,1),outShapes(:,2),'xb');
% for j = 1:size(centers,1)
%     if interiorShapes(j) == 0 % ~=1
%     text(centers(j,1),centers(j,2),int2str(j),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','b');
%     end
% end

%find the neighbors of the allowed triangles
safeCells = [1:size(DT,1)]';
safeCells = safeCells(interiorShapes == 0); % ~= 1);
safeCenters = centers(interiorShapes == 0,:); % ~= 1,:);
N = neighbors(DT);
safeN = N(interiorShapes == 0,:); % ~= 1,:);
%create an adjacency matrix with only links between legit nodes
centersT = centers.';
 adjMrx = nan(size(N,1),size(N,1)); %zeros(size(N,1),size(N,1));
 for i = 1:size(N,1)
     if interiorShapes(i) == 0 % ~= 1
         for j = 1:size(N,2)
             if ~isnan(N(i,j)) && interiorShapes(N(i,j)) == 0 % ~= 1
                 adjMrx(i,N(i,j)) = findDist(centersT(:,i),centersT(:,N(i,j))); % 1;
                 adjMrx(N(i,j),i) = findDist(centersT(:,i),centersT(:,N(i,j))); % 1;
             end
         end
     end
 end
safeAdjMrx = adjMrx(interiorShapes == 0,interiorShapes == 0); %adjMrx(interiorShapes ~= 1,interiorShapes ~= 1);

start = [currPose(1,1) currPose(2,1)];
% goal = [2.2 2.1]; % [1 -.5]; % [4 -2.5];
RM = [safeCenters safeAdjMrx];

% figure(2)
% [path pDist PRM] = findPath( verticies,RM,start,goal,maxX,minX,maxY,minY);
% shortestPath = path;
% pathDist = pDist;

RM = beefRM( polygons,RM,maxX,minX,maxY,minY); %beefRM( verticies,RM,maxX,minX,maxY,minY);

shortestPath = NaN;
pathDist = NaN;


if (isempty(unvisitedWPs))
else
    numWps = size(unvisitedWPs,2);
    for i = 1:numWps
        goal = unvisitedWPs(:,i);
        goal = goal.';
        [path pDist] = findPath( polygons,RM,start,goal,maxX,minX,maxY,minY); %findPath( verticies,RM,start,goal,maxX,minX,maxY,minY);
        
        if isnan(pathDist)
            pathDist = pDist;
            shortestPath = path;
            ptIdx = i;
        elseif (pDist < pathDist)
            shortestPath = path;
            pathDist = pDist;
            ptIdx = i;
        end
    end
end
waypointPath = shortestPath;
wpPathDist = pathDist;
wpIdx = ptIdx;

shortestPath = NaN;
pathDist = NaN;
ptIdx = NaN;
    
if (isempty(optWalls));
else
    checkWallPts = [];
    for j = 1:size(optWalls,1)
        buff = .31;
        x1 = optWalls(j,1);
        y1 = optWalls(j,2);
        x2 = optWalls(j,3);
        y2 = optWalls(j,4);
        x = [x1 x2];
        y = [y1 y2];
%         hold on
%         plot(x1,y1,'r-');
%         hold off
        theta = atan2((y1 - y2),(x1 - x2));
        wallCtrX = (x1 + x2)/2;
        wallCtrY = (y1 + y2)/2;
        xOffset = buff*sin(theta);
        yOffset = -buff*cos(theta);
        
        newX1 = wallCtrX + xOffset;
        newY1 = wallCtrY + yOffset;
        newX2 = wallCtrX - xOffset;
        newY2 = wallCtrY - yOffset;
        
        cwp = [newX1 newX2; newY1 newY2; wallCtrX wallCtrX; wallCtrY wallCtrY; j j];
        checkWallPts = [checkWallPts cwp];
    end
    
    numPts = size(checkWallPts,2);
    for i = 1:numPts
        goal = checkWallPts(1:2,i);
        goal = goal.';
        [path pDist] = findPath( polygons,RM,start,goal,maxX,minX,maxY,minY); %findPath( verticies,RM,start,goal,maxX,minX,maxY,minY);
        
        if isnan(pathDist)
            pathDist = pDist;
            shortestPath = path;
            ptIdx = i;
        elseif (pDist < pathDist)
            shortestPath = path;
            pathDist = pDist;
            ptIdx = i;
        end
    end
    if isnan(path)
        ptIdx = NaN;
    else
        ind = find(checkWallPts(5,:) == checkWallPts(5,ptIdx));
        if (ind(1,1) == ptIdx)
            lastPt = ind(2);
        elseif (ind(1,2) == ptIdx)
            lastPt = ind(1);
        end
        shortestPath = [shortestPath; 0 checkWallPts(1,lastPt) checkWallPts(2,lastPt)];
        pathDist = pathDist + buff;
        ptIdx = checkWallPts(5,ptIdx);
        wpOrWall = 2;
    end   
end

checkShortest = [wpPathDist; pathDist];
[val idx] = min(checkShortest);
if (idx == 1)
    pathDist = wpPathDist;
    shortestPath = waypointPath;
    ptIdx = wpIdx;
    wpOrWall = 1;
end
    
    makePolyMap( polygons,minX,maxX,minY,maxY );
    figure(2)
    hold on
    plot(shortestPath(:,2),shortestPath(:,3),'g-','LineWidth',3);
    hold off

end

