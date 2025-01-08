function errormap = updateErrormap(plannedLikelihood,executedLikelihood,errormap)
% UPDATEERRORMAP Use difference between predicted and executed trajectories
% to update a map of trajectory errors.
%   errormap = UPDATEERRORMAP(plannedLikelihood,executedLikelihood,errormap)
%   uses informationa about the planned and executed trajectories, stored
%   in their respective likelihoods, to update the errormap. Nonzero values
%   correspond to locations where the two trajectories deviated from one 
%   another; larger values correspond to bigger deviations. Positive values
%   denote locations where the executed trajectory was successful (and are
%   thus candidates for sampling new anchor points); negative values denote
%   locations where the planned trajectory was unsuccessful (and thus
%   correspond to regions that should be avoided in the future).
%
%   See also: EVALUATETRAJECTORY

errormap = errormap + executedLikelihood(:,:,1) - plannedLikelihood(:,:,1);

end