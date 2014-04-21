function [ v, w ] = feedbackLin( v_x, v_y, theta, epsilon)
%UNTITLED7 Summary of this function goes here
%   INPUTS
%  v_x, v_y - desired velocity in the inertial frame
%  theta - heading of the robot wrt inertial frame
%  turn radius

b_R_n = [cos(theta) sin(theta); - sin(theta) cos(theta)];
eMatrix = [1 0; 0 1/epsilon];

V_out = eMatrix*b_R_n*[v_x;v_y];
v = V_out(1);
w = V_out(2);



end

