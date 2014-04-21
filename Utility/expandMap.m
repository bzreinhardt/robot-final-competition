function [newEnv,newBorder] = expandMap(env,border,R)
%EXPANDMAP bloats the walls of a polygonal environment to avoid collisions
%for a robot with a radius
%INPUTS
% env - a matrix defining the obstacles in the enviroment. each row
%    represnets an obstacle in the form [x1,y1,x2,y2...] with each vertex
%        connected to the next and the last vertex is connected to the first
%    if the obstacles are of different sizes, the row is padded with zeros
%   verticies are in counterclockwise order
% border - verticies of the polygonal border [x1,y1, x2,y2 ...] with each 
%     vertex connected to the next and the last connected to the first [1x2M]
% verticies are in counterclockwise order
%R - collision avoidance radius (recommended to be the robot's radius + a
%   bit extra)
%OUTPUTS
%newEnv - new nodes/shapes in the same format as the input env
%newBorder - new border nodes in the same format as the input border

verticies = env;
%break shapes down into list of x y points
coords =cell(size(verticies,1),1);%yCoords = cell(size(verticies,1),1);
for i = 1:size(verticies,1)
    for j = 1:2:size(verticies,2)
        if (verticies(i,j) ~= 0 && verticies(i,j+1) ~= 0)
            coords{i} = [coords{i},[verticies(i,j);verticies(i,j+1)]]; 
            %yCoords{i} = [yCoords{i};verticies(i,j+1)];
        end
        
    end
end
newCoords = cell(size(verticies,1),1);
%go through each shape 
for i = 1:size(verticies,1)
    nodes = coords{i}; newNodes = zeros(2,3*size(nodes,2));
    %cycle through each corner of the shape
    for j = 1:size(nodes,2)
        %find the prevous corner and next corner
        if j == 1
            prev = nodes(:,end);
        else
            prev = nodes(:,j-1);
        end
        n = nodes(:,j);
        if j == size(nodes,2)
            next = nodes(:,1);
        else
            next = nodes(:,j+1);
        end
        %find the direction to expand on the previous side
        prevExp = [0 1;-1 0]* (n - prev)/(norm(n-prev));
        nextExp = [0 1;-1 0]* (next-n)/(norm(next-n));
        %expand the node in the directions perpendicular to the two walls
        %it connects
        n1 = n + prevExp*R;
        n3 = n + nextExp*R;
        
        %find the direction directly out from the corner
        outExp = ((n-prev)+(n-next))/norm((n-prev)+(n-next));
        %test for concavity - if it's concave, subtract the direction out
        
        if (n(2)-prev(2))*(n(1)-next(1))>=(n(1)-prev(1))*(n(2)-next(2))
            %convex
            n2 = n + outExp*2^0.5*R;
        else
            %if it's concave, just draw a line between the expanded edges
            theta = acos(((n-prev)'*(n-next))/(norm(n-prev)*norm(n-next)));
            len = R/(sin(theta/2));
            n2 = n - len*((n-next)+(n-prev))/norm((n-next)+(n-prev));
            n1 = n2;
            n3 = n2;
        end
        %put these three new nodes into the new nodes
        newNodes(:,3*j-2:3*j) = [n1,n2,n3];
    end
    newCoords{i} = newNodes;
end
%put the new shapes into standard enviornemnt format
newEnv = []; maxNodes = 3*size(verticies,2);
for i = 1:size(verticies,1)
    newShape = zeros(1,2*size(newCoords{i},2));
    for j = 1:size(newCoords{i},2)
        newShape(1,2*j-1:2*j) = newCoords{i}(:,j)'; 
    end
    %pad out the shape 
    padShape = padarray(newShape,[0 maxNodes-size(newShape,2)],'post');
    newEnv = [newEnv;padShape];
end

%do the same thing for the border, but expand it inwards
%find the verticies of the border
borderCoords = [];
for j = 1:2:size(border,2)
    
        borderCoords = [borderCoords,[border(1,j);border(1,j+1)]];
   
    
end
squeezeCoords = zeros(2,3*size(borderCoords,2));
    %cycle through each corner of the shape
    for j = 1:size(borderCoords,2)
        %find the prevous corner and next corner
        if j == 1
            prev = borderCoords(:,end);
        else
            prev = borderCoords(:,j-1);
        end
        n = borderCoords(:,j);
        if j == size(borderCoords,2)
            next = borderCoords(:,1);
        else
            next = borderCoords(:,j+1);
        end
        %find the direction to expand on the previous side
        prevExp = [0 1;-1 0]* (n - prev)/(norm(n-prev));
        nextExp = [0 1;-1 0]* (next-n)/(norm(next-n));
        %expand the node in the directions perpendicular to the two walls
        %it connects
        n1 = n - prevExp*R;
        n3 = n - nextExp*R;
        
        %find the direction directly out from the corner
        outExp = ((n-prev)+(n-next))/norm((n-prev)+(n-next));
        %test for concavity - if it's concave, subtract the direction out
        
        if (n(2)-prev(2))*(n(1)-next(1))>(n(1)-prev(1))*(n(2)-next(2))
            %convex
            n2 = n - outExp*2^0.5*R;
        else
            %if it's concave, just draw a line between the expanded edges
            theta = acos(((n-prev)'*(n-next))/(norm(n-prev)*norm(n-next)));
            len = R/(sin(theta/2));
            n2 = n + len*((n-next)+(n-prev))/norm((n-next)+(n-prev));
            n1 = n2;
            n3 = n2;
        end
        %put these three new nodes into the new nodes
        squeezeCoords(:,3*j-2:3*j) = [n1,n2,n3];
    end
%put the points into a single line
    newBorder=reshape(squeezeCoords,[1,numel(squeezeCoords)]);
   

end