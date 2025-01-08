function plannedTrajectory = evaluateTrajectory(trajectory,errormap,belief,sampler,planner)
% EVALUATETRAJECTORY Determine whether planned trajectory is likely to 
% intercept an obstacle, and readjust if necessary.
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
    %sample new anchor points from the errormap
    [thAnchorsAug,rAnchorsAug] = sampleAnchors(errormap,belief,sampler);

    % remove home port from existing set of anchor points, and augment 
    % set with newly sampled anchor points
    thAnchors = [trajectory.thAnchors(2:end-1),thAnchorsAug];
    rAnchors  = [trajectory.rAnchors( 2:end-1),rAnchorsAug ];

    % plan new trajectory through augmented set of anchors
    plannedTrajectory = optimizeTrajectory(thAnchors,rAnchors,belief,planner);

%otherwise, proceed with initial plan for trajectory
else
    plannedTrajectory = trajectory;
end

end