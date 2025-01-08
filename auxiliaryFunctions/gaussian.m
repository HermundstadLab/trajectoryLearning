function f = gaussian(x,y,x0,y0,sigX,sigY)
% GAUSSIAN Evaluate a 2D Gaussian function. 
%   f = GAUSSIAN(x,y,x0,y0,sigX,sigY) evaluates a gaussian with mean 
%   [x0,y0] and standard deviation [sigX,sigY] over the cartesian input 
%   arrays x and y. The arrays x and y must have the same size.

f = exp( -(x-x0).^2 / (2*sigX.^2) - (y-y0).^2 / (2*sigY.^2) );
end