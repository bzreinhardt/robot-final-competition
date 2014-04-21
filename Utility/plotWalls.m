function plotWalls( walls )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
for k = 1:length(walls(:,1))
    line([walls(k,1) walls(k,3)],[walls(k,2) walls(k,4)],'Color','r');
end

end

