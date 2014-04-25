%testing Particle filter with beacons and sonars

%initialize particles
load('ExampleMap1_2014.mat');
cameraR = 0;
sonarR = 0.16;
Q_sonar = 0.01;
cov_AR = 0.005;
R = 0.1;
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
pf = [];
M = 100;
X_0 = dataStore.truthPose(1,2:4)'*ones(1,M)+randn(3,M);

figure(1);clf; 
xlim([min(map(:,1)),max(map(:,3))]);
ylim([min(map(:,2)),max(map(:,4))]);
plot(dataStore.truthPose(1,2),dataStore.truthPose(1,3),'x','MarkerSize',10); hold on;
% quiver(X_0(1,:),X_0(2,:),0.5*cos(X_0(3,:)),0.5*sin(X_0(3,:)));
% plot(X_0(1,:),X_0(2,:),'ro');
quiver(dataStore.truthPose(1,2),dataStore.truthPose(1,3),0.5*cos(dataStore.truthPose(1,4)),...
    0.5*sin(dataStore.truthPose(1,4)));


for i = 1:size(dataStore.truthPose,1)
    
    if i == 1
        X_in = X_0;
    else
        delete(handle1);delete(handle2);
    end
    X = dataStore.truthPose(i,2:4)';
    meas = dataStore.measurements(i,1:find(dataStore.measurements(i,:),1,'last'));
    ARs = dataStore.ARs(i,1:find(dataStore.ARs(i,:),1,'last'));
    sonars = dataStore.sonars(i,1:find(dataStore.sonars(i,:),1,'last'));
    p_z = @(X,z)pfWeightSonarAR(X,z,h,ARs,sonars,Q_sonar,Q_AR);
    weight = [weight;feval(p_z,X,meas)];
    [X_out,w_out] =  particleFilter(X_in,meas,dataStore.odometry(i,2:3)',g,p_z,R);
    pf = [pf;X_out;w_out];
    X_in = X_out;
    handle1 = quiver(X_out(1,:),X_out(2,:),0.5*cos(X_out(3,:)),0.5*sin(X_out(3,:)));
    handle2 = plot(X_out(1,:),X_out(2,:),'go');
    xlim([min(map(:,1)),max(map(:,3))]);
    ylim([min(map(:,2)),max(map(:,4))]);
    drawnow
    
end
    




%Test PF



% for i = 1:size(X_out,2)
%     text(X_out(1,i),X_out(2,i),num2str(w_out(i)));
% end