function width = getPeakWidth(np,height,maxWidth)
% GETPEAKWIDTH Determine the approximate width of a normalized 2D gaussian 
% peak, given its height and level of discretization.
%
%   sigma = GETPEAKWIDTH(np,height,sigmaMax) finds the width (defined here
%   to be 10% of the standard deviation) of a normalized 2D gaussian peak 
%   with a given input height. The width scales linearly with 1/sqrt(height); 
%   the scaling depends on the number of points ('np') that are used to 
%   discretize the gaussian (this assumes that the gaussian is defined over 
%   a space of size np x np) The maximum width is bounded by 'maxWidth'.

% compute the width of a gaussian peak of given height (the scaling 0.036
% was determined by fitting the standard deviation of a 2D gaussian
% distribution as a function of its height and discretization):
a = 0.036./np;
width = a./sqrt(height);

% bound maximum width
width(width>maxWidth) = maxWidth;

end