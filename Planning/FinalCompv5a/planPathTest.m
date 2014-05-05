function [ output_args ] = planPathTest( input_args )
%UNTITLED16 Summary of this function goes here
%   Detailed explanation goes here

map = load('ExampleMap3_2014.mat');
currPose = [-5.5; 5]; % [4.5; -1.5; 0]; % [3; -3; 0]; % [-4; 4; 0];


walls = map.map;
optWalls = map.optWalls;

optWalls = optWalls(1,:);

% optWalls = [];
wpOrWall = 1;

regWps = map.waypoints;
regWps = regWps.';
ecWps = map.ECwaypoints;
ecWps = ecWps.';
wps = [regWps ecWps];

wps = [-.5; -3.5];

unvisitedWPs = wps;
visitedWPs = [];

[shortestPath pathDist relocalize ptIdx] = planPath(walls,optWalls,wpOrWall,currPose,unvisitedWPs,visitedWPs );

end

