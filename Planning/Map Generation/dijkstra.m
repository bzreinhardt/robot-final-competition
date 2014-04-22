function [distance,S] = dijkstra(graph, source,target,ignore)
%DIJKSTRA finds the shortest distance from a source node to a target node
% based on pseudocode on wikipedia: 
% http://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
%INPUTS
%graph - adjacency matrix of the graph NxN where N is the number of nodes.
% graph(i,j) is the weight of the edge between i and j if they are
% connected and 0 otherwise.
%source - source node - some number between 1 and N
%target - target node - some number between 1 and N
%ignore - set of nodes to ignore (such as known unconnected nodes)
%OUTPUTS
%dist - distance along path 
%S - path of nodes between the source and target node


dist = inf * ones(1,size(graph,1)); %initialize distances to infinity
previous = zeros(1,size(graph,1)); %initialize previous nodes for each node

dist(source) = 0;                   %distance to source is 0 durr
Q = 1:size(graph,1);                %put all nodes in graph into Q

while ~isempty(Q) && ~isempty(setdiff(Q,ignore))
    %find and pull the closest node from Q
    d_sub = dist(Q);
    [min_d_sub, ind_d_sub] = min(d_sub(:));
    [ypeak_sub, xpeak_sub] = ind2sub(size(d_sub), ind_d_sub);
    u = Q(xpeak_sub);
     
    Q = Q(Q~=u);
    if u == target && dist(u) ~= Inf    %if you pulled the target node, you win!
        break;
    end
    %if all the nodes are infinitely far away, 
        % the graph isn't connected :-( - return an empty path and an
        % infinte distance
    if dist(u) == Inf               
        S = [];
        distance = Inf;
        return;
    end
    neighbors = find(graph(u,:));   %get a list of the neighbors of u
    for i = 1:length(neighbors)     %iterate over neighbors of u
        v = neighbors(i);
        alt = dist(u) + graph(u,v); 
        if alt < dist(v);           %if this is a shorter path to v, change
            dist(v) = alt;
            previous(v) = u;
                                    
        end
    end
end
% go backwards to find the shortest path to the target and its length
u = target;
distance = 0;
S = [];
while previous(u) ~= 0 
    S = [u; S]; %put u on the path to the target
    distance = distance + graph(u,previous(u));
    u = previous(u);
end
S = [u;S]; %put the first node on the path
end