load('ExampleMap2_2014')
mapLims = [min([map(:,1);map(:,3)]),min([map(:,2);map(:,4)]),...
    max([map(:,1);map(:,3)]),max([map(:,2);map(:,4)])];
figure(1); 
wallVisualizer(map, optWalls, beaconLoc, waypoints, ECwaypoints)
xlim([mapLims(1),mapLims(3)]);ylim([mapLims(2),mapLims(4)]); 
cameraR = 0;
C_beacon = [1.4;-0.76];
beaconNum = 3;
M = 30;
G_beacon = beaconLoc(beaconLoc(:,1) == beaconNum,2:3)';
[relocParts] = beaconRelocalize(G_beacon,C_beacon,M,mapLims,cameraR);
plot(G_beacon(1),G_beacon(2),'go');
quiver(relocParts(1,:),relocParts(2,:),0.1*cos(relocParts(3,:)),0.1*sin(relocParts(3,:)));