function traj = boundTrajectory(traj,xBounds,yBounds)
% BOUNDTRAJECTORY Returns a new trajectory whose spatial extent does not 
% exceed a set of bounds.
%
%   traj = BOUNDTRAJECTORY(traj,xBounds,yBounds) takes as input a trajectory
%   and set of bounds defined in cartesian coordinates, returns a trajectory
%   that is constrained within those bounds.

traj.xCoords(traj.xCoords<xBounds(1)) = xBounds(1);
traj.xCoords(traj.xCoords>xBounds(2)) = xBounds(2);
traj.yCoords(traj.yCoords<yBounds(1)) = yBounds(1);
traj.yCoords(traj.yCoords>yBounds(2)) = yBounds(2);
end