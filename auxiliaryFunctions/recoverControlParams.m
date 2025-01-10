function [hCoords,vCoords] = recoverControlParams(trajectory)
% RECOVERCONTROLPARAMS Recovers the underlying control parameters from a
% cartesian trajectory.
%   [vCoords,hCoords] = RECOVERCONTROLPARAMS(trajectory) takes a trajectory
%   (with cartesian coordinates [trajectory.xCoords, trajectory.yCoords])
%   as input, and returns the instantaneous heading ('hCoords') and speed 
%   ('vCoord') profiles that generated the trajectory.
%
%   See also: GENERATETRAJECTORYSEGMENT 

% get instantaneous heading
[thCoords,rCoords] = cart2pol(trajectory.xCoords,trajectory.yCoords);
[dth,~] = dpol(thCoords,rCoords);
hCoords = wrapTo2Pi(dth);

% get instanteous speed
vCoords = sqrt(diff(trajectory.xCoords).^2 + diff(trajectory.yCoords).^2);


end