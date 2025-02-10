function [xBoundary,yBoundary,heading,indSegments,binSegments,corners] = getBoundary(xBounds,yBounds,np)
% GETBOUNDARY Returns densely-sampled coordinates of rectangular area.
%   [xBounds,yBounds,heading,indSegments,binSegments,corners] = GETBOUNDS(xBounds,yBounds,np) 
%   uses np points per unit distance to interpolate each of the boundaries 
%   specified by xBounds = [xmin,xmax] and yBounds = [ymin,ymax], and 
%   returns the x- and y-coordinates of the boundary, together with the 
%   heading of a trajectory that traverses the same boundary. Individual
%   boundaries are specified by the indices in 'indSegments'; boundary IDs
%   are specified by 'binSegments', and corner positions by the nonzero
%   entries in 'corners'.

xBoundary = [linspace(xBounds(1),xBounds(2),floor(np*diff(xBounds))),...
    linspace(xBounds(2),xBounds(2),floor(np*diff(yBounds))),...
    linspace(xBounds(2),xBounds(1),floor(np*diff(xBounds))),...
    linspace(xBounds(1),xBounds(1),floor(np*diff(yBounds)))];

yBoundary = [linspace(yBounds(1),yBounds(1),floor(np*diff(xBounds))),...
    linspace(yBounds(1),yBounds(2),floor(np*diff(yBounds))),...
    linspace(yBounds(2),yBounds(2),floor(np*diff(xBounds))),...
    linspace(yBounds(2),yBounds(1),floor(np*diff(yBounds)))];

heading = [zeros(1,floor(np*diff(xBounds))),...
    pi/2*ones(   1,floor(np*diff(yBounds))),...
    pi*ones(     1,floor(np*diff(xBounds))),...
    3*pi/2*ones( 1,floor(np*diff(yBounds)))];

bounds = cumsum([0,floor(np*diff(xBounds)),floor(np*diff(yBounds)),...
    floor(np*diff(xBounds)),floor(np*diff(yBounds))]);

indSegments = cell(1,4);
binSegments = [];
for i=1:4
    indSegments{i} = bounds(i)+1:bounds(i+1);
    binSegments = [binSegments,bounds(i)+1];
end
corners = zeros(size(xBoundary));
corners(binSegments) = 1;
binSegments = [binSegments,bounds(end)+1];

end