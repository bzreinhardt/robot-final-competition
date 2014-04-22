%Main Script for Robots HW 7
%% Delanay Cell Decomposition

verticies = dlmread('hw7.txt');
boundary = [0 0 0 100 100 100 100 0];
env = verticies;
border = boundary;
x = []; y = []; shape = []; constraints = [];
k = 1; shapeStart = 1;
for i = 1:size(verticies,1)
    shapeStart = k;
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
DT = delaunayTriangulation(x,y,constraints);
figure(1);clf;triplot(DT); hold on
%% Find the triangles that are actually in shapes
% plot the actual obstacles    
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end
%find the delaunay edges that go through the shapes and mark them
E = edges(DT);
nodes = DT.Points;
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

plot(midX,midY,'x'); hold on
inObs = [];
onObs = [];
for i = 1:max(shape)
    [IN ON] = inpolygon(midX,midY,x(shape == i),y(shape == i));
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
inX = midX(interiorEdges == 1);
inY = midY(interiorEdges == 1);
plot(inX,inY,'gx');

graphE = E(~interiorEdges,:);
%adjMrxS = sparse([graphE(:,1);graphE(:,2)],[graphE(:,2);graphE(:,1)],ones(2*size(graphE,1),1),size(graphE,1),size(graphE,1));
adjMrxB = zeros(size(nodes,1),size(nodes,1));
for i = 1:size(graphE,1)
    adjMrxB(graphE(i,1),graphE(i,2)) = 1;
    adjMrxB(graphE(i,2),graphE(i,1)) = 1;
end
%plot the resulting thing
figure(2);
gplot(adjMrxB,nodes,'-o');
for j = 1:size(nodes,1),
    text(nodes(j,1),nodes(j,2),int2str(j),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','r');
end
%%
%another way of figuring out what to get rid of - if the center of the
%triangle is in the same place as an object
inObs2 = [];
centers = incenter(DT,[1:size(DT,1)]');
for i = 1:max(shape)
    [IN ON] = inpolygon(centers(:,1),centers(:,2),x(shape == i),y(shape == i));
    inObs2 = [inObs2,IN];
    %onObs = [onObs,ON];
end
interiorShapes = sum(inObs2,2);
%plot the 
figure(3);clf;triplot(DT); hold on
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end
%plot(centers(:,1),centers(:,2),'x'); hold on
inShapes = centers(interiorShapes == 1,:);
outShapes = centers(interiorShapes == 0,:);
plot(inShapes(:,1),inShapes(:,2),'xg');
for j = 1:size(centers,1)
    if interiorShapes(j) ~=1
    text(centers(j,1),centers(j,2),int2str(j),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','b');
    end
end

%find the neighbors of the allowed triangles
safeCells = [1:size(DT,1)]';
safeCells = safeCells(interiorShapes ~= 1);
safeCenters = centers(interiorShapes ~= 1,:);
N = neighbors(DT);
safeN = N(interiorShapes ~= 1,:);
%create an adjacency matrix with only links between legit nodes
 adjMrx = zeros(size(N,1),size(N,1));
 for i = 1:size(N,1)
     if interiorShapes(i) ~= 1
         for j = 1:size(N,2)
             if ~isnan(N(i,j)) && interiorShapes(N(i,j)) ~= 1
                 adjMrx(i,N(i,j)) = 1;
                 adjMrx(N(i,j),i) = 1;
             end
         end
     end
 end
safeAdjMrx = adjMrx(interiorShapes ~= 1,interiorShapes ~= 1);

 
 figure(4);clf;
gplot(adjMrx,centers,'-o');
for j = 1:size(centers,1)
    if interiorShapes(j) ~=1
    text(centers(j,1)+1,centers(j,2)+1,int2str(j),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','b');
    end
end

figure(5);clf;
gplot(safeAdjMrx,safeCenters,'-o');
for j = 1:size(safeCenters,1)
   
    text(safeCenters(j,1)+1,safeCenters(j,2)+1,int2str(j),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','b');
   
end

[centers, adjMrx, DT2, intShapes] = createRoadmap(env, border);
[I,J,V] = find(adjMrx);
midX1 = centers(1,I); 
midX2 = centers(1,J);
midY1 = centers(2,I); 
midY2 = centers(2,J);
midX3 = abs(midX1-midX2)/2+min([midX1;midX2],[],1);
midY3 = abs(midY1-midY2)/2+min([midY1;midY2],[],1);



figure(6);clf;
gplot(adjMrx,centers','-o');
for j = 1:size(centers',1)
   if intShapes(j) ~=1
    text(centers(1,j)+1,centers(2,j)+1,int2str(j),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','b');
   end
end
for i = 1:size(midX3,2)
    text(midX3(i),midY3(i),num2str(V(i)),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','k');
end

%% Generating figures for findPath
init1 = [22;65]; goal1 = [90;10];
init2 = [80;95]; goal2 = [10;50];

[path1, totDist1, startNode1, endNode1] = findPath(verticies, centers, adjMrx, init1, goal1);
[path2, totDist2, startNode2, endNode2] = findPath(verticies, centers, adjMrx, init2, goal2);

figure(7); clf;
%plot the paths
plot(path1(1,:),path1(2,:),'-.g','LineWidth',5); hold on
plot(path2(1,:),path2(2,:),'-k','LineWidth',5); hold on
%plot the roadmap
gplot(adjMrx,centers','-o');
for j = 1:size(centers',1)
   if intShapes(j) ~=1
    text(centers(1,j)+1,centers(2,j)+1,int2str(j),'FontSize',14,'FontWeight','bold','FontName','Courier','Color','b');
   end
end
%plot the obstacles
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end


%plot the endpoints
text(path1(1,1),path1(2,1),'Path 1 Start'); 
text(path1(1,end),path1(2,end),'Path 1 Goal');
text(path2(1,1)+1,path2(2,1)+1,'Path 2 Start'); 
text(path2(1,end)+1,path2(2,end)+1,'Path 2 Goal');
legend('Path 1','Path 2');

%% Generating Probabilistic roadmaps
n = 10;
%uniform random sampling
samplingFcn1 = @uniformSample;
[PRMnodes1,PRMadjMrx1] = buildPRM(env, border, n, samplingFcn1);
%plot the result
figure(8);clf;
plot(init1(1),init1(2),'rx','MarkerSize',20); hold on
plot(goal1(1),goal1(2),'kx','MarkerSize',20); hold on
legend('Start','Goal');
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end

gplot(PRMadjMrx1,PRMnodes1','-o');
[path1, totDist1, startNode1, endNode1] = ...
    findPath(env, PRMnodes1, PRMadjMrx1, init1, goal1);
if ~isempty(path1)
    plot(path1(1,:),path1(2,:),'-k','LineWidth',5); hold on
end
%low dispersion sampling
samplingFcn2 = @lowDispSample;
[PRMnodes2,PRMadjMrx2] = buildPRM(env, border, n, samplingFcn2);
figure(9);clf;
plot(init1(1),init1(2),'rx','MarkerSize',20); hold on
plot(goal1(1),goal1(2),'kx','MarkerSize',20); hold on
legend('Start','Goal');
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end

gplot(PRMadjMrx2,PRMnodes2','-o');
[path2, totDist2, startNode2, endNode2] = ...
    findPath(env, PRMnodes1, PRMadjMrx1, init1, goal1);

%keep making random roadmaps until you get a path between the start and the
%end
goodPathRand = []; n1 = 9;
while isempty(goodPathRand)
    n1 = n1+1;
    [PRMnodes1,PRMadjMrx1] = buildPRM(env, border, n1, samplingFcn1);
    [goodPathRand, totDistRand, startNodeRand, endNodeRand] = ...
    findPath(env, PRMnodes1, PRMadjMrx1, init1, goal1);
end
figure(10);clf;
plot(init1(1),init1(2),'rx','MarkerSize',20); hold on
plot(goal1(1),goal1(2),'kx','MarkerSize',20); hold on
legend('Start','Goal');
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end

gplot(PRMadjMrx1,PRMnodes1','-o');

    plot(goodPathRand(1,:),goodPathRand(2,:),'-k','LineWidth',5); hold on
    title(strcat('n = ',num2str(n1)));
    
%keep making low disp roadmaps until you get a path between the start and the
%end
goodPathDisp = []; n2 = 9;
while isempty(goodPathDisp)
    n2 = n2+1;
    [PRMnodes2,PRMadjMrx2] = buildPRM(env, border, n2, samplingFcn2);
    [goodPathDisp, totDistDisp, startNodeDisp, endNodeDisp] = ...
    findPath(env, PRMnodes2, PRMadjMrx2, init1, goal1);
end
figure(11);clf;
plot(init1(1),init1(2),'rx','MarkerSize',20); hold on
plot(goal1(1),goal1(2),'kx','MarkerSize',20); hold on
legend('Start','Goal');
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end

gplot(PRMadjMrx2,PRMnodes2','-o');

    plot(goodPathDisp(1,:),goodPathDisp(2,:),'-k','LineWidth',5); hold on
    title(strcat('n = ',num2str(n2)));
    
    
    %build a visibility sample
%     n3 = 9;
%     samplingFcn3 = @visSample;
%     [nodes3,adjMrx3] = buildPRM(env, border, n3, samplingFcn3);
%     
%     figure(12);clf;
%     
% plot(init1(1),init1(2),'rx','MarkerSize',20); hold on
% plot(goal1(1),goal1(2),'kx','MarkerSize',20); hold on
% legend('Start','Goal');
% for i = 1:max(shape)
%     patch(x(shape == i),y(shape == i),'r');
%     hold on
% end
% plot(nodes3(1,:),nodes3(2,:),'x'); hold on;
% 
% gplot(adjMrx3,nodes3','-o'); hold on

goodPathVis = []; n3 = 4;
while isempty(goodPathVis)
    n3 = n3+1
    [PRMnodes3,PRMadjMrx3] = buildPRM(env, border, n3, samplingFcn3);
    [goodPathVis, totDistVis, startNodeVis, endNodeVis] = ...
    findPath(env, PRMnodes3, PRMadjMrx3, init1, goal1);
end
figure(12);clf;
plot(init1(1),init1(2),'rx','MarkerSize',20); hold on
plot(goal1(1),goal1(2),'kx','MarkerSize',20); hold on
legend('Start','Goal');
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end

gplot(PRMadjMrx3,PRMnodes3','-o');

    plot(goodPathVis(1,:),goodPathVis(2,:),'-k','LineWidth',5); hold on
    title(strcat('n = ',num2str(n3)));
    
 % plot(PRMnodes3(1,:),PRMnodes3(2,:),'o'); hold on;  
%% Testing RRT
startRRT = [20; 65]; goalRRT = [90;10];
figure(13);clf;
plot(startRRT(1),startRRT(2),'rx','MarkerSize',20); hold on
plot(goalRRT(1),goalRRT(2),'kx','MarkerSize',20); hold on
legend('Start','Goal');
for i = 1:max(shape)
    patch(x(shape == i),y(shape == i),'r');
    hold on
end

[wayPts,wayMrx,points] = buildRRTpoint(env,border,startRRT,goalRRT);
plot(wayPts(1,:),wayPts(2,:),'k','LineWidth',3);


