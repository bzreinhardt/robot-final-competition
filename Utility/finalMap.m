function finalMap(dataStore, mapFile)
%final map plots the known walls, the robots trajectory and visited
%waypoints
figure(11);
clf;
load(mapFile);
map = dataStore.walls;
optWalls = [];
mapLims = [min([map(:,1);map(:,3)]),min([map(:,2);map(:,4)]),...
    max([map(:,1);map(:,3)]),max([map(:,2);map(:,4)])];
wallVisualizer(map, optWalls, beaconLoc, waypoints, ECwaypoints); hold on
trajectory = plot(dataStore.X(:,1),dataStore.X(:,2),'g');
wayPts = plot(dataStore.wayPtsVisited(:,1),dataStore.wayPtsVisited(:,2),'rx','MarkerSize',20);
xlim([mapLims(1),mapLims(3)]);ylim([mapLims(2),mapLims(4)]);
end
