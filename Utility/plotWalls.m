function plotWalls( walls,varargin )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
if nargin > 1
    color = varargin{1};
else
    color = 'r';
end

for k = 1:length(walls(:,1))
    line([walls(k,1) walls(k,3)],[walls(k,2) walls(k,4)],'Color',color);
end

end

