%this is a script to automate as much of the robot setup and calibration as
%possible

%a) get a robot
%b) robot[ID].coecis.cornell.edu
%c) get AR tag
ARtag = input('enter robot AR tag number: ');
ID = input('enter robot ID number: ');

%login via putty
robotName = strcat('robot',num2str(ID),'.coecis.cornell.edu');
login = 'beagle';
password = 'cornell';
cmd1 = 'cd beagoleoncreate';
cmd2 = 'sudo ./main';
%enter password again

nextStep = input('successfully connected to robot?' );

ports = CreateBeagleInit(ID);
CreatePort = ports.create;
BeaglePort = ports.beagle;
SonarPort = ports.sonar;
BeaconPort = ports.beacon;
BeepRoomba(CreatePort);

CalibGUI(ports);



%%
% Calibrate sonars
% Click on the Calibrate Left/Front/Right Sonar button to start the calibration process for of the corresponding sonar. The calibration GUI will then ask you to place a card board in front of the corresponding sonar at 4 known distances.
% Click Next button when you have placed the card board at the corresponding distance.
% When the sonar reading is NaN, it will ask you to click Next again when the reading is good.
% After 4 distances are measured for the sonar, Click Done Calibration to finish sonar calibration. Three calibration offsets are calculated by the mean of the four measurement offsets with each sonar.
% The SONAR_OFFSET variable is automatically saved to a file named sonar_calibration.mat so that you can load it in the future.
% Calibrate ARtag beacon
% The process of calibrating ARtag beacon is similar to calibrate sonars. Make sure the lighting in the room is good so that ARtag can be easily detectable.
% 
% Click on the Calibrate Camera Button to start the calibration process. The calibration GUI will then ask you to place an ARtag in front of the camera at 4 known distances. Make sure there is only one ARtag in the camera's view.
% Click Next button when you have placed the ARtag at the corresponding distance.
% When there is no ARtag detected in the view, it will ask you to click Next again when there is ARtag detected.
% After 4 distances are measured, Click Done Calibration to finish beacon calibration. An calibration offset is calculated by the mean of the four measurement offsets.
% The BEACON_OFFSET variable is automatically saved to a file named beacon_calibration.mat so that you can load it in the future.