function [intersect,intersection,inds] = intersectTrajectory(xCoords,yCoords,xBounds,yBounds)
% INTERSECTTRAJECTORY Determines whether a set of trajectory coordinates
% intersects a rectangular bounded region. 
%
%   [intersect,intersection,inds] = INTERSECTTRAJECTORY(xCoords,yCoords,xBounds,yBounds) 
%   takes as input the cartesian coordinates of a trajectory segment, and
%   the cartesian bounds of a rectangular region. It then determines 
%   whether the segment intersected the region ('intersect'; a binary
%   output), and returns the indices and coordinates of the intersecting
%   portion(s) of the segment (in 'inds' and 'intersection', respectively)
%
%   See also: BOUNDTRAJECTORY

inds = find(xCoords>=xBounds(1) & xCoords<=xBounds(2) ...
& yCoords>=yBounds(1) & yCoords<=yBounds(2));

if numel(inds)>0
    intersect = 1;
else
    intersect = 0;
end
intersection = zeros(size(xCoords));
intersection(inds) = 1;
end