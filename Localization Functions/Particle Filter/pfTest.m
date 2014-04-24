%testing Particle filter with beacons and sonars

%initialize particles
load('ExampleMap1_2014.mat');
cameraR = 0;
sonarR = 0.16;
Q_sonar = 0.01;
cov_AR = 0.001;
R = 0.01;
theta = 0.1;
%rotM = [cos(theta) -sin(theta);sin(theta) cos(theta)];
Q_AR = cov_AR*eye(2);
%function to predict measurement based on map

%function to predict position based on odometry data
g = @integrateOdom;
h = @(X,ARs,sonars)hBeaconSonar(X,ARs,sonars,map,beaconLoc,cameraR,sonarR);
%weighting function
p_z = @(X,z)pfWeightSonarAR(X,z,h,ARs,sonars,Q_sonar,Q_AR);

weight = [];

for i = 1:size(dataStore.truthPose,1)
    X = dataStore.truthPose(i,2:4)';
    meas = dataStore.measurements(i,1:find(dataStore.measurements(i,:),1,'last'));
    ARs = dataStore.ARs(i,1:find(dataStore.ARs(i,:),1,'last'));
    sonars = dataStore.sonars(i,1:find(dataStore.sonars(i,:),1,'last'));
    p_z = @(X,z)pfWeightSonarAR(X,z,h,ARs,sonars,Q_sonar,Q_AR);
    weight = [weight;feval(p_z,X,meas)];
end
    
X_0 = dataStore.truthPose(1,2:4)'*ones(1,20)+randn(3,20);
figure(1);clf;
plot(dataStore.truthPose(1,2),dataStore.truthPose(1,3),'x','MarkerSize',10); hold on;
plot(X_0(1,:),X_0(2,:),'ro');
%TODO propigate in time and make sure its working
[X_out,w_out] = particleFilter(X_0,meas,dataStore.odometry(end,2:3)',g,p_z,R);
plot(X_out(1,:),X_out(2,:),'go');
for i = 1:size(X_out,2)
    text(X_out(1,i),X_out(2,i),num2str(w_out(i)));
end