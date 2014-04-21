function [DT, occTris] = genDT(env,border)
%GENDT generates a delaunay composition of a polygonal region
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
%find the triangles that are occupied by polygons
inObs2 = [];
centers = incenter(DT,[1:size(DT,1)]');
for i = 1:max(shape)
    [IN ON] = inpolygon(centers(:,1),centers(:,2),x(shape == i),y(shape == i));
    inObs2 = [inObs2,IN];
    %onObs = [onObs,ON];
end
occTris = sum(inObs2,2);
end