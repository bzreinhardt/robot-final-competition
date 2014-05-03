function [ output_args ] = plotGraph( nodeArray,edgeGraph,polyFile,mapXMin,mapXDim,mapYMin,mapYDim )
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here

figure(2)
makePolyMap(polyFile,mapXMin,mapXDim,mapYMin,mapYDim);
hold on
scatter(nodeArray(:,1),nodeArray(:,2),'ro');
for i = 1:size(edgeGraph,1)
    for j = (i+1):size(edgeGraph,2)
        if isnan(edgeGraph(i,j)) || (edgeGraph(i,j) == 0);
        else
            x = [nodeArray(i,1) nodeArray(j,1)];
            y = [nodeArray(i,2) nodeArray(j,2)];
            plot(x,y);
        end
    end
end
hold off

end

