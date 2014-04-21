function [noSing] =  removeSing(interpData)
%Takes a column vector interpData, checks for interpolated singularities
%and corrects them
    for i = 1:length(interpData(:,1))-2;
        if abs(interpData(i,1)-interpData(i+2,1)) > 5.5;
            %example data 0.1; 4; 6
            if interpData(i,1) > 5.5
                interpData(i,1) = interpData(i,1)-2*pi;
            elseif interpData(i+2,1) > 5.5
                interpData(i+2,1) = interpData(i,1)-2*pi;
            end
            %assume points are evenly spaced
            interpData(i+1,1) = (interpData(i+2,1)-interpData(i,1))/2;
        end
        
    end
    noSing = interpData;
end