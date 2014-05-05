function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,robotRad)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,ROBOTRAD) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       robotRad    robot radius (in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #1
%   O'Brien, Kevin

rightWheel = (robotRad*angVel) + fwdVel;
leftWheel = fwdVel - (robotRad*angVel);
rightWheelAbs = abs(rightWheel);
rightWheelNew = rightWheelAbs;
leftWheelAbs = abs(leftWheel);
leftWheelNew = leftWheelAbs;
if rightWheelAbs < .000001
    rightWheelSign = 1;
else
    rightWheelSign = rightWheel/rightWheelAbs;
end
if leftWheelAbs < .000001
    leftWheelSign = 1;
else
    leftWheelSign = leftWheel/leftWheelAbs;
end
if rightWheelAbs > maxV
    rightWheelNew = maxV*rightWheelSign;
    if leftWheelAbs < .000001
        leftWheelNew = 0;
    else
        leftWheelNew = (leftWheel*maxV)/rightWheelAbs;
    end
end
if abs(leftWheelNew) > maxV
    leftWheelFinal = maxV*leftWheelSign;
    if rightWheelAbs < .000001
        rightWheelFinal = 0;
    else
        rightWheelFinal = abs((rightWheelNew*maxV)/leftWheelNew)*rightWheelSign;
    end
else
    leftWheelFinal = abs(leftWheelNew)*leftWheelSign;
    rightWheelFinal = abs(rightWheelNew)*rightWheelSign;
end
angVel = (rightWheelFinal - leftWheelFinal)/(2*robotRad);
if abs(angVel) < .001
    angVel = 0;
end
cmdV = (leftWheelFinal + rightWheelFinal)/2;
cmdW = angVel;
    

