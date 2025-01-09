function [dr,dx,dy] = dcart(xvec,yvec)
% DCART Compute distance between points, using cartesian inputs. 
%   [dr,dx,dy] = DCART(xvec,yvec) computes the distance between successive
%   elements in the 1D arrays xvec, yvec. The arrays xvec and yvec must have 
%   the same size; if of size [1,n], the output arrays dr, dx, dy will be of
%   size [1,n-1].
%
% See also: DPOL

dx = diff(xvec);
dy = diff(yvec);
dr = sqrt( dx.^2 + dy.^2 ); 
end