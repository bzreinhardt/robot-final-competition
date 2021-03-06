function [ v, w ] = feedbackLin( v_x, v_y, theta, epsilon)
%UNTITLED7 Summary of this function goes here
%   INPUTS
%  v_x, v_y - desired velocity in the inertial frame
%  theta - heading of the robot wrt inertial frame
%  turn radius
okErr = 0.2;
if epsilon == 0
    %if you are pointed in approximately the right direction, go forward
    normXY = [v_x;v_y]/norm([v_x;v_y]);
    if abs(cos(theta)-normXY(1))+abs(sin(theta)-normXY(2))<okErr;
        v = norm([v_x;v_y]);
        w = 0;
        %otherwise turn slowly
    else
        theta2 = mod(2*pi+atan2(v_y,v_x),2*pi);
        v = 0;
        if theta2 >= theta
            if theta2 - theta <=pi
                %turn left
                w = norm([v_x;v_y]);
            else
                %turn right
                w = -norm([v_x;v_y]);
            end
        else
            if theta - theta2 <= pi
                %turn right
                w = -norm([v_x;v_y]);
            else
                %turn left
                w = norm([v_x;v_y]);
            end
            
        end
    end
        
else
    
    

b_R_n = [cos(theta) sin(theta); - sin(theta) cos(theta)];
eMatrix = [1 0; 0 1/epsilon];

V_out = eMatrix*b_R_n*[v_x;v_y];
v = V_out(1);
w = V_out(2);
end



end

