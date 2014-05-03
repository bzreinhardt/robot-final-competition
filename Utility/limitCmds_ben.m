function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,Wheel2center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,Wheel2center) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       Wheel2center    robot radius (in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #1
%   REINHARDT, BENJAMIN 

%check the intended velocity of each wheel
v_r = fwdVel + angVel * Wheel2center;
v_l = fwdVel - angVel * Wheel2center;
if abs(v_r) > maxV || abs(v_l) > maxV
    %scale commanded velocities if they would saturate motors
    if fwdVel == 0
        cmdV = 0;
        cmdW = maxV/Wheel2center*sign(angVel);
        return; 
    end
    ratio = abs(angVel/fwdVel);
    cmdV = maxV/(1 + ratio*Wheel2center)*sign(fwdVel);
    cmdW = ratio * abs(cmdV) * sign(angVel);
else
    %pass commands through if they will not saturate the motors
    cmdV = fwdVel;
    cmdW = angVel;
end






