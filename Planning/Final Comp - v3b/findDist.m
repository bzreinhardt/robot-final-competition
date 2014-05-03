function [distK] = findDist(ctr1,ctr2)
    ctr1X = ctr1(1,1);
    ctr1Y = ctr1(2,1);
    ctr2X = ctr2(1,1);
    ctr2Y = ctr2(2,1);
    distK = sqrt((ctr1X - ctr2X)^2 + (ctr1Y - ctr2Y)^2);
end