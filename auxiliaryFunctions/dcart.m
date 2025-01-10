function dr = dcart(xvec,yvec)
% DCART Compute distance between points, using cartesian inputs. 
%   dr = DCART(xvec,yvec) computes the distance between successive
%   elements in the 1D arrays xvec, yvec. The arrays xvec and yvec must  
%   have the same size; if of size [1,n], the output array dr will be of
%   size [1,n-1].
%
% See also: DPOL

dr = sqrt( diff(xvec).^2 + diff(yvec).^2 ); 
end