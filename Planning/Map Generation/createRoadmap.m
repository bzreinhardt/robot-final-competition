function [centers, adjMrx,DT,interiorShapes] = createRoadmap(env, border)
% CREATEROADMAP creates a map representing a roadmap through a 2D enviornment
% comprising polygonal obstacles and a plygonal boundary
% 
% INPUTS
% env - a matrix defining the obstacles in the enviroment. each row
%    represnets an obstacle in the form [x1,y1,x2,y2...] with each vertex
%        connected to the next and the last vertex is connected to the first
%    if the obstacles are of different sizes, the row is padded with zeros
% border - verticies of the polygonal border [x1,y1, x2,y2 ...] with each 
%     vertex connected to the next and the last connected to the first [1xM]
% OUTPUTS
% centers - matrix of the centers for the triangles comprising the nodes of
%     the roadmap. [Nx2] where N is the number of nodes
% adjMrx - matrix of adjacencies between nodes on the roadmap. [NxN] matrix
%     where adjMrx(i,j) = 1 if node i and node j connect and 0 otherwise

%initialize containters and counters
x = []; y = []; shape = []; constraints = []; 
k = 1; shapeStart = 1;

verticies = env;
%add verticies to vertex lists and edges to constraints list
for i = 1:size(verticies,1)
    shapeStart = k;
    for j = 1:2:size(verticies,2)
        if (verticies(i,j) ~= 0 && verticies(i,j+1) ~= 0)
            x = [x;verticies(i,j)]; 
            y = [y;verticies(i,j+1)];
            shape = [shape;i];
            constraints = [constraints; k k+1];
            
            if j+2 > size(verticies,2) || (verticies(i,j+2) == 0 && verticies(i,j+3) == 0 && j>3)
                constraints(end,:) = [k shapeStart];
            end
            k = k + 1;
        end
        
    end
end
%add the border to the list of verticies
for i = 1:2:size(border,2)
    x = [x;border(i)];
    y = [y;border(i+1)];
    shape = [shape;0]; 
end
%generate triangulation for the map
DT = delaunayTriangulation(x,y,constraints);
%find the centers of the triangle
allCenters = incenter(DT,[1:size(DT,1)]');
inObs = [];

for i = 1:max(shape)
    [IN ON] = inpolygon(allCenters(:,1),allCenters(:,2),x(shape == i),y(shape == i));
    inObs = [inObs,IN];
end
% triangles that are inside an obstacle are forbidden
interiorShapes = sum(inObs,2);
% find the centers and adjacency matrix for the roadmap (eliminating stuff
% inside 

centers = allCenters';
N = neighbors(DT);

adjMrx = zeros(size(N,1),size(N,1));
 for i = 1:size(N,1)
     if interiorShapes(i) ~= 1
         for j = 1:size(N,2)
             if ~isnan(N(i,j)) && interiorShapes(N(i,j)) ~= 1
                 adjMrx(i,N(i,j)) = findDist(centers(:,i),centers(:,N(i,j)));
                 adjMrx(N(i,j),i) = findDist(centers(:,i),centers(:,N(i,j)));
             end
         end
     end
 end
% safeAdjMrx = adjMrx(interiorShapes ~= 1,interiorShapes ~= 1);
% 
% adjMrx = safeAdjMrx;

end