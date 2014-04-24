beacon13 = beaconLoc(12,:);
beacon15 = beaconLoc(14,:);
n = size(dataStore.truthPose,1);
exp13 = zeros(2,n);
exp15 = zeros(2,n);
robotRad = 0.13;

son1 = -pi/2;
son2 = 0;
son3 = pi/3;
exp1 = zeros(n,1);
exp2 = zeros(n,1);
exp3 = zeros(n,1);

for i = 1:n
    exp13(:,i) = hBeacon(dataStore.truthPose(i,2:4), beacon13, robotRad);
    exp15(:,i) = hBeacon(dataStore.truthPose(i,2:4), beacon15, robotRad);
    
    exp1(i) = sonarPredict(dataStore.truthPose(i,2:4)',map,robotRad,son1,2);
    exp2(i) = sonarPredict(dataStore.truthPose(i,2:4)',map,robotRad,son2,2);
    exp3(i) = sonarPredict(dataStore.truthPose(i,2:4)',map,robotRad,son3,2);

    
end



