function plannedTrajectory = evaluateTrajectory(trajectory,errormap,belief,sampler,planner)
% EVALUATETRAJECTORY Determine whether planned trajectory is likely to 
% intercept an obstacle, and readjust if necessary.
%
%   plannedTrajectory = EVALUATETRAJECTORY(trajectory,errormap,belief,...
%   sampler,planner) takes as input a planned trajectory and a previously
%   evaluated errormap to determine whether the planned trajectory is
%   predicted to generate high errors because of an interferring obstacle.
%   If this predicted error exceeds a threshold (specified in the 'sampler'
%   input structure), this function uses parameters stored within the 
%   'sampler' and 'planner' structures to sample anchors points from the
%   errormap, augment the existing anchors points with sampled ones, and
%   plan a new trajectory through the augmented anchors.
% 
%   See also: UPDATEERRORMAP, SAMPLEANCHORS, PLANTRAJECTORY, OPTIMIZETRAJECTORY

% evaluate the highest predicted error of planned trajectory (high errors
% correspond to large negative values of the errormap)
[thCoords,rCoords] = cart2pol(trajectory.xCoords,trajectory.yCoords);
[~,indth] = min(abs(thCoords'-belief.thAxes),[],2);
[~,indr]  = min(abs(rCoords' -belief.rAxes), [],2);
lininds   = sub2ind(size(errormap),indr,indth);
predError = min(errormap(lininds));

% if error exceeds threshold, augment the set of anchor points by sampling
% from the errormap, and plan a new trajectory
if predError<sampler.errorThreshold
    % sample new 'augmenting' set of anchor points from the errormap
    anchorsAug = sampleAnchors(errormap,belief,sampler,planner);

    % remove home port from existing set of anchor points, and augment 
    % set with newly sampled anchor points
    anchors.thCoords = [trajectory.anchors.thCoords(2:end-1),anchorsAug.thCoords                      ];
    anchors.rCoords  = [trajectory.anchors.rCoords( 2:end-1),anchorsAug.rCoords                       ];
    anchors.thTol    = [trajectory.anchors.thTol(   2:end-1),planner.thTol_shift.*ones(1,anchorsAug.N)];
    anchors.rTol     = [trajectory.anchors.rTol(    2:end-1),planner.rTol_shift.*ones( 1,anchorsAug.N)];
    anchors.N = numel(anchors.thCoords);

    % plan new trajectory through augmented set of anchors
    plannedTrajectory = optimizeTrajectory(anchors,belief,planner);

%otherwise, proceed with initial plan for trajectory
else
    plannedTrajectory = trajectory;
end

end