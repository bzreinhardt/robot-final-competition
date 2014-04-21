%futzing script to visualize the example maps
load('ExampleMap1_2014.mat');
figure(1);clf;
plot(waypoints(:,1),waypoints(:,2),'x','MarkerSize',10,'Color','r'); hold on;
plot(ECwaypoints(:,1),ECwaypoints(:,2),'x','MarkerSize',10,'Color','g');
plot(beaconLoc(:,2),beaconLoc(:,3),'o','MarkerSize',10);

plotWalls(optWalls,'g');
plotWalls(map);
legend('waypoints','EC waypoints','beaconLocations','optional walls');
title('Example Map 1');

load('ExampleMap2_2014.mat');
figure(2);clf;
plot(waypoints(:,1),waypoints(:,2),'x','MarkerSize',10,'Color','r'); hold on;
plot(ECwaypoints(:,1),ECwaypoints(:,2),'x','MarkerSize',10,'Color','g');
plot(beaconLoc(:,2),beaconLoc(:,3),'o','MarkerSize',10);

plotWalls(optWalls,'g');
plotWalls(map);
legend('waypoints','EC waypoints','beaconLocations','optional walls');
title('Example Map 2');

load('ExampleMap3_2014.mat');
figure(3);clf;
plot(waypoints(:,1),waypoints(:,2),'x','MarkerSize',10,'Color','r'); hold on;
plot(ECwaypoints(:,1),ECwaypoints(:,2),'x','MarkerSize',10,'Color','g');
plot(beaconLoc(:,2),beaconLoc(:,3),'o','MarkerSize',10);

plotWalls(optWalls,'g');
plotWalls(map);
legend('waypoints','EC waypoints','beaconLocations','optional walls');
title('Example Map 3');
