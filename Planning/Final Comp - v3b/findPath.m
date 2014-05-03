function [ shortestPath pathDist PRM] = findPath( polyFile,PRM,start,goal,mapXDim,mapXMin,mapYDim,mapYMin)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

numConnections = 3;
buffer = 0;

numNodes = size(PRM,1);

nanBuffer = NaN(numNodes,1).';
addStart = [start nanBuffer];
addGoal = [goal nanBuffer];
PRM = [PRM; addStart; addGoal];

numNodes = size(PRM,1);
addStart = NaN(numNodes,1);
addGoal = NaN(numNodes,1);
PRM = [PRM addStart addGoal];

nodeArray = PRM(:,1:2);
edgeGraphNew = PRM(:,3:end);

for i = (numNodes - 1):numNodes % i = 1:size(edgeGraphNew,2)
    N = edgeGraphNew(:,i);
    freeN = find(~isnan(N));
    if size(freeN,1) < numConnections
        
        % find closest node to start
        flag = 0;
        m = 0;
        currNode = PRM(i,1:2);
        aX = nodeArray(:,1) - currNode(:,1);
        aY = nodeArray(:,2) - currNode(:,2);
        c = hypot(aX,aY);
        c(i,1) = NaN;
        while ((flag < 3) && (m < 10))
            [val idx] = min(c);
            if isnan(val)
                % flag = 1;
                break
            end
            testX = nodeArray(idx,1);
            testY = nodeArray(idx,2);
%             x = [startX testX];
%             y = [startY testY];
%             plot(x,y,'g-');
            freePath = isPathFree1( polyFile,currNode(:,1),currNode(:,2),testX,testY ); % isPathFreeMap( polyFile,buffer,currNode(:,1),currNode(:,2),testX,testY ); %
            if (freePath == 1)
                flag = flag + 1;
                c(idx,1) = NaN;
                PRM(i,(idx + 2)) = val; % PRM(idx,(size(PRM,2) - 1)) = val;
                PRM(idx,(i + 2)) = val;
            end
            m = m + 1;
        end
        
    end
    
end

nodeArray = PRM(:,1:2);
edgeGraphNew = PRM(:,3:end);
plotGraph(nodeArray,edgeGraphNew,polyFile,mapXMin,mapXDim,mapYMin,mapYDim);

[shortestPath pathDist] = dijkstras(PRM);
end

