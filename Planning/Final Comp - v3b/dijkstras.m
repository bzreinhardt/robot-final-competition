function [ shortestPath pathDist ] = dijkstras( PRM )
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here

numNodes = size(PRM,1);

% complete the PRM
eGraph = PRM(:,3:end);
for i = 1:numNodes
    for j = 1:numNodes
%         if eGraph(i,j) == 0
%             eGraph(i,j) = NaN;
%         end
        eGraph(j,i) = eGraph(i,j);
    end
end
nodesArray = PRM(:,1:2);
PRM = [nodesArray eGraph];

% initialize the visited and unvisited arrays
visited = []; % [(numNodes - 1) 0];
for i = 1:numNodes
    unvisited(i,1) = i+2;
    unvisited(i,2) = NaN;
end
unvisited((numNodes - 1),2) = 0;
% unvisited((numNodes - 1),1) = NaN;

flag = 0;
node = 0;
while ((node ~= (numNodes + 2)) ||  (flag == 1))%insert while loop here
    weights = unvisited(:,2);
    [weight index] = min(weights);
    if isnan(weight)
        flag = 1;
        break;
    end
    node = unvisited(index,1);
    neighbors = [];
    for i = 1:numNodes
        if (isnan(PRM(i,node)))
        else
            hasBeenVisited = 0;
            for j = 1:size(visited,1)
                if (visited(j,1) == (i + 2))
                    hasBeenVisited = 1;
                    break
                end
            end
            if (hasBeenVisited == 0)
                neighbor = [i PRM(i,node)];
                neighbors = [neighbors; neighbor];
            end
        end
    end

    for i = 1:size(neighbors,1)
        neighborIndex = neighbors(i,1);
        neighborWeight = neighbors(i,2);
        if (((weight + neighborWeight) < unvisited(neighborIndex,2)) || (isnan(unvisited(neighborIndex,2))))
        unvisited(neighborIndex,2) = weight + neighborWeight;
        unvisited(neighborIndex,3) = node;
        end
    end
    
    visited = [visited; node weight];
    unvisited((node - 2),1) = NaN;
    unvisited((node - 2),2) = NaN;
    
end % insert end for while loop : end

if (flag == 0 )
    goal = numNodes + 2;
    node = goal;
    reversePath = node;
    while (( node ~= (numNodes + 1) ) || (isnan(node)) || (size(reversePath,1) == numNodes))
        nextNode = unvisited((node - 2),3);
        reversePath = [reversePath nextNode];
        node = nextNode;
    end
    
    shortestPath = fliplr(reversePath);
    shortestPath = shortestPath.';
    for i = 1:size(shortestPath,1)
        shortestPath(i,2) = PRM((shortestPath(i,1) - 2),1);
        shortestPath(i,3) = PRM((shortestPath(i,1) - 2),2);
    end
    figure(2)
    hold on
    plot(shortestPath(:,2),shortestPath(:,3),'r-','LineWidth',3);
    hold off
    pathDist = visited(end,2);
else
    shortestPath = NaN;
    pathDist = NaN;
end
    
end

