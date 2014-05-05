function[maxX maxY minX minY] = findMaxMin(map)
mx1 = map(:,1);
my1 = map(:,2);
mx2 = map(:,3);
my2 = map(:,4);
maxX1 = max(mx1);
maxX2 = max(mx2);
minX1 = min(mx1);
minX2 = min(mx2);
maxY1 = max(my1);
maxY2 = max(my2);
minY1 = min(my1);
minY2 = min(my2);
maxX = max(maxX1,maxX2);
maxY = max(maxY1,maxY2);
minX = min(minX1,minX2);
minY = min(minY1,minY2);
end