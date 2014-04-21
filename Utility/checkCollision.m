function [collision,wall,pt] = checkCollision(env,p1,p2)
%CHECKCOLLISION checks if a line between p1 and p2 will collide with any
%walls in a polygonal environment 
%INPUTS 
%env - a matrix defining the obstacles in the enviroment. each row
%    represnets an obstacle in the form [x1,y1,x2,y2...] with each vertex
%        connected to the next and the last vertex is connected to the first
%    if the obstacles are of different sizes, the row is padded with zeros
% p1 - start point [2x1]
% p2 - end point [2x1]
%OUTPUTS
% collision - 1 if there is a collision, 0 otherwise
% wall - endpoints of a wall causing the collision, empty otherwise
% pt - point of the collision, empty otherwise
collision = 0; wall = []; pt = [];
for i = 1:size(env,1)
    for j = 1:2:size(env,2)
        if (env(i,j) ~= 0 && env(i,j+1) ~= 0)
            %find the verticies of the wall
            v1 = env(i,j:j+1)';
            
            if j+2 > size(env,2) || (env(i,j+2) == 0 && env(i,j+3) == 0 && j>3)
                v2 = env(i,1:2)';
            else
                v2 = env(i,j+2:j+3)';
            end
        end
        [isect,x,y,ua]= intersectPoint(p1(1),p1(2),p2(1),p2(2)...
            ,v1(1),v1(2),v2(1),v2(2));
        if isect == 1
            collision = 1;
            wall = [v1',v2'];
            pt = [x;y];
            break
        end
    end
end


