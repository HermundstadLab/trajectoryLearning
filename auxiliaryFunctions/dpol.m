function [dth,dr] = dpol(thvec,rvec)
% DPOL Compute the distance between points, using polar inputs. 
%   [dr,dx,dy] = DPOL(xvec,yvec) computes the angular and radial distance 
%   between successive elements in the 1D arrays thvec (radians) and rvec. 
%   The arrays thvec and rvec must have the same size; if of size [1,n], 
%   the output arrays dth (radians) and dr will be of size [1,n-1].
%
% See also: DCART

dth = atan2(rvec(2:end).*sin(thvec(2:end))-rvec(1:end-1).*sin(thvec(1:end-1)),...
    rvec(2:end).*cos(thvec(2:end))-rvec(1:end-1).*cos(thvec(1:end-1))); 
dr  = sqrt(rvec(1:end-1).^2 + rvec(2:end).^2 - 2.*rvec(1:end-1).*rvec(2:end).*cos(thvec(2:end)-thvec(1:end-1)));
end