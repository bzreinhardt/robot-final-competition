%futzing script to visualize the example maps
function wallVisualizer(map, optWalls, beaconLoc, waypoints, ECwaypoints)

plot(waypoints(:,1),waypoints(:,2),'S','MarkerSize',10,'Color','k'); hold on;
if ~isempty(ECwaypoints)
plot(ECwaypoints(:,1),ECwaypoints(:,2),'x','MarkerSize',10,'Color','k');
end
plot(beaconLoc(:,2),beaconLoc(:,3),'o','MarkerSize',5);
for i = 1:size(beaconLoc,1)
    text(beaconLoc(i,2),beaconLoc(i,3),num2str(beaconLoc(i,1)));
end

plotWalls(optWalls,'--');
plotWalls(map);
%legend('waypoints','EC waypoints','beaconLocations','optional walls');



end