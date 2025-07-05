function [dth,dr] = dpol(thvec,rvec)
% DPOL Compute the distance between points, using polar inputs. 
%   [dth,dr] = DPOL(thvec,rvec) computes the angular and radial distance 
%   between successive elements in the 1D arrays thvec (radians) and rvec. 
%   The arrays thvec and rvec must have the same size; if of size [1,n], 
%   the output arrays dth (radians) and dr will be of size [1,n-1].
%
% See also: DCART

dth = atan2(diff(rvec.*sin(thvec)),diff(rvec.*cos(thvec)));
dr  = sqrt(diff(rvec).^2 + 2.*rvec(1:end-1).*rvec(2:end).*(1-cos(diff(thvec)))); 
end