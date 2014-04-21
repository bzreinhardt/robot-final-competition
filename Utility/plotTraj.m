function [handle] = plotTraj(dataStore,varargin)
%plots the trajectory of a normal robot datastore object
%INPUTS 
%dataStore - datstore structure
%optional - data you want to plot (functionality to be added later)
if nargin == 1
    handle = plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3));
end