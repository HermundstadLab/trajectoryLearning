function executedTrajectory = executeTrajectory(trajectory,arena,trial,planner,trialID)
% EXECUTETRAJECTORY Execute planned trajectory, accounting for arena
% boundaries and obstacles.
%
%   executedTrajectory = EXECUTETRAJECTORY(trajectory,arena,trial,planner,trialID)
%   takes as input a planned trajectory, and executes a version of the
%   trajectory that is consistent with arena boundaries and obstacles.
%   General and trial-specific properties of arena and obstacle boundaries
%   are conveyed through the input structures 'arena' and 'trial',
%   respectively.
%
%   See also: PLANTRAJECTORY


% determine portions of trajectory that pass through obstacle
[~,obstacleVec,~] = intersectTrajectory(trajectory.xCoords,trajectory.yCoords,...
    trial.obstacle.xBounds(trial.blockIDs(trialID),:),...
    trial.obstacle.yBounds(trial.blockIDs(trialID),:));

% find timepoints when trajectory enters and exits obstacle
entrances = find(diff(obstacleVec)>0);
exits     = find(diff(obstacleVec)<0)+1;

% remove cases where the agent nicks the corner of an obstacle
%START HERE

% specify entry angles of obstable boundaries, indexed by:
% boundary ID: 1:bottom; 2:right; 3:top; 4:left (rows of array)
% orientation: 1:CW; 2:CCW (columns of array)
angles = [[0, pi,   ];...
   [    pi/2, 3*pi/2];...
   [      pi, 2*pi  ];...
   [  3*pi/2, pi/2  ]];

% consider each planned trajectory that intersects the obstacle 
% (separated into 'boundary' and 'open field' components); these will be
% inserted into the planned trajectory
boundarySegments = {};
openSegments     = {};

blockID = trial.blockIDs(trialID);
for i=1:numel(entrances)
    
    % extract segment of trajectory that passes through obstacle
    inds = entrances(i):exits(i);                   % indices of segment
    xseg = trajectory.xCoords(inds);                % x-coordinates of segment
    yseg = trajectory.yCoords(inds);                % y-coordinates of segment
    hseg = wrapTo2Pi(trajectory.heading(inds));     % heading during segment

    % determine which obstacle boundary the trajectory entered:
    % 1:bottom; 2:right; 3:top; 4:left
    [~,indStart] = min( abs(xseg(1)-trial.obstacle.xBoundary(blockID,:)) ...
        + abs(yseg(1)-trial.obstacle.yBoundary(blockID,:)) );
    boundaryID   = find(histcounts(indStart,trial.obstacle.indCorners(blockID,:)));

    % determine the travel direction that minimizes the change in heading
    [~,orientation] = min(abs(hseg(1) - angles(boundaryID,:)));

    % extract oriented segment of boundary along which trajectory will be diverted
    xBoundary  = circshift(trial.obstacle.xBoundary(blockID,:),numel(trial.obstacle.xBoundary(blockID,:))-indStart+1);
    yBoundary  = circshift(trial.obstacle.yBoundary(blockID,:),numel(trial.obstacle.yBoundary(blockID,:))-indStart+1);
    heading    = circshift(trial.obstacle.heading(blockID,:),  numel(trial.obstacle.heading(  blockID,:))-indStart+1);
    if orientation>1
        xBoundary = fliplr(xBoundary);
        yBoundary = fliplr(yBoundary);
        heading   = fliplr(heading);
        heading   = wrapTo2Pi(heading+pi);
    end
    [~,indEnd] = min(abs(xseg(end)-xBoundary) + abs(yseg(end)-yBoundary));
    xBoundary = xBoundary(1:indEnd);
    yBoundary = yBoundary(1:indEnd);
    heading   = heading(  1:indEnd);

    % extract first anchor point after trajectory passes through obstacle
    prevAnchorID = trajectory.prevAnchor(exits(i));
    nextAnchorID = prevAnchorID+1;
    [xAnchor,yAnchor] = pol2cart(trajectory.anchors.thCoords(nextAnchorID),...
        trajectory.anchors.rCoords(nextAnchorID));

    % if next anchor is within the obstacle, select the subsequent anchor
    if intersectTrajectory(xAnchor,yAnchor,trial.obstacle.xBounds(blockID,:),trial.obstacle.yBounds(blockID,:))
        nextAnchorID = nextAnchorID+1;
        [xAnchor,yAnchor] = pol2cart(trajectory.anchors.thCoords(nextAnchorID),...
            trajectory.anchors.rCoords(nextAnchorID));
    end

    % extract the obstacle faces that are exposed to the anchor point
    [xExposedFace,yExposedFace,exposedFaceID] = getSolidAngle(trial,trialID,xAnchor,yAnchor);

    % find the point along the boundary segment that first intersects the
    % exposed faces of the obstacle (the first entry in 'indExposed')
    [~,indExposed,~] = intersect([xBoundary',yBoundary'],...
        [xExposedFace',yExposedFace'],'rows','stable');

    % compute velocity along boundary (to be stored below)
    boundaryVelocity = max([abs(xBoundary(1:2)),...
        abs(diff(yBoundary(1:2)))]).*planner.nInterp;

    % extract index of next anchor point
    nextAnchorIndex = find(trajectory.prevAnchor==nextAnchorID,1,'first');

    % if the agent is already on the exposed face, or if the exposed face 
    % is not contained in the set of boundaries that the agent traverses
    % along its diverted trajectory, assume the agent executes the entire
    % diverted trajectory until it reaches its original exit point from
    % the obstacle
    
    if ismember(boundaryID,exposedFaceID) || numel(indExposed)==0
        inds = 1:numel(xBoundary);
        trajDivertedBoundary.prevAnchor = prevAnchorID.*ones(size(inds));
        trajDivertedBoundary.xCoords    = xBoundary;
        trajDivertedBoundary.yCoords    = yBoundary;
        trajDivertedBoundary.velocity   = boundaryVelocity.*ones(size(inds));
        trajDivertedBoundary.heading    = heading;

        inds = exits(i)+1:nextAnchorIndex;
        trajDivertedOpen.prevAnchor     = prevAnchorID.*ones(size(inds));
        trajDivertedOpen.xCoords        = trajectory.xCoords( inds);
        trajDivertedOpen.yCoords        = trajectory.yCoords( inds);
        trajDivertedOpen.velocity       = trajectory.velocity(inds);
        trajDivertedOpen.heading        = trajectory.heading( inds);

    % otherwise, assume the agent travels around obstacle boundary until it
    % can peel away toward next anchor point
    else

        % generate the first portion of the diverted trajectory by moving at 
        % an approximately constant speed around the unexposed portion of the 
        % obstacle:
        inds = 1:indExposed(1);
        trajDivertedBoundary.prevAnchor = prevAnchorID.*ones(size(inds));
        trajDivertedBoundary.xCoords    = xBoundary(inds);
        trajDivertedBoundary.yCoords    = yBoundary(inds);
        trajDivertedBoundary.velocity   = boundaryVelocity.*ones(size(inds));
        trajDivertedBoundary.heading    = heading(inds);

        % once trajectory reaches the first exposed corner of the obstacle,
        % use the corner as an anchor point, and generate a standard trajectory 
        % segment to the next (i.e., the original) anchor point:
        [anchors.thCoords,anchors.rCoords] = cart2pol(xBoundary(indExposed(1)+1),yBoundary(indExposed(1)+1));
        anchors.thCoords = [anchors.thCoords,trajectory.anchors.thCoords(nextAnchorID)];
        anchors.rCoords  = [anchors.rCoords, trajectory.anchors.rCoords( nextAnchorID)];
    
        % choose initial heading such that agent initially follows obstacle 
        % boundary before peeling away; if this generates a large discontinuity
        % in heading at next anchor point, choose the negative of this heading. 
        nextBoundaryHeading = heading(inds(end)+1);
        [dth,~] = dpol(anchors.thCoords,anchors.rCoords);
        phi = nextBoundaryHeading-dth+pi/2;
        phiSet = [phi,-phi];
    
        % if the next anchor point is the final one (i.e., the home port), peel
        % away from the boundary (i.e., choose first of allowed initial headings)
        if nextAnchorID==max(trajectory.prevAnchor)+1
            phi = phiSet(1);

        % otherwise, choose the initial heading that minimizes the difference
        % with the subsequent heading
        else
            angleVec = [-2*pi,0,2*pi]';
            nextAnchorHeading = trajectory.heading(nextAnchorIndex);
            finalHeading = pi/2+dth-phiSet;
            [~,indMin] = min(min(finalHeading-(nextAnchorHeading-angleVec),[],1));
            phi = phiSet(indMin);
        end
       
        trajDivertedOpen = planTrajectory(anchors,phi,planner);
        trajDivertedOpen.prevAnchor = trajDivertedOpen.prevAnchor+prevAnchorID-1;
    end

    % store diverted trajectory segments to be inserted into planned trajectory:
    boundarySegments{i} = trajDivertedBoundary;
    openSegments{i}     = trajDivertedOpen;

end

% insert diverted trajectory segments into planned trajectory
trajFields = {'prevAnchor','xCoords','yCoords','velocity','heading'};
if numel(entrances)>0
    % store trajectory segment until first intersection with obstacle
    for i=1:numel(trajFields)
        executedTrajectory.(trajFields{i}) = trajectory.(trajFields{i})(1:entrances(1)-1);
    end

    % intert trajectory segments that were diverted around obstacle
    for i=1:numel(entrances)-1
        nextAnchorID = boundarySegments{i}.prevAnchor(1)+1;
        nextAnchorIndex = find(trajectory.prevAnchor==nextAnchorID,1,'first');
        for j=1:numel(trajFields)
            executedTrajectory.(trajFields{j}) = ...
                [executedTrajectory.(trajFields{j}),...                         % existing trajectory up until obstacle entrance
                boundarySegments{i}.(trajFields{j}),...                         % diverted trajectory: boundary segment
                openSegments{i}.(trajFields{j}),...                             % diverted trajectory: open segment
                trajectory.(trajFields{j})(nextAnchorIndex:entrances(i+1)-1)];  % remaining trajectory to next obstacle entrance
        end
    end

    % insert final diverted segment, and append segment after last 
    % interaction with obstacle
    nextAnchorID = boundarySegments{end}.prevAnchor(1)+1;
    nextAnchorIndex = find(trajectory.prevAnchor==nextAnchorID,1,'first');
    for i=1:numel(trajFields)
        executedTrajectory.(trajFields{i}) = ...
            [executedTrajectory.(trajFields{i}),...
            boundarySegments{end}.(trajFields{i}),...
            openSegments{end}.(trajFields{i}),...
            trajectory.(trajFields{i})(nextAnchorIndex:numel(trajectory.(trajFields{i})))];
    end

    % store remaining fields:
    trajFields = {'amplitude','offset','anchors'};
    for i=1:numel(trajFields)
        executedTrajectory.(trajFields{i}) = trajectory.(trajFields{i});
    end

else
    executedTrajectory = trajectory;
end

% bound portions of trajectory that exceed edges of arena
executedTrajectory = boundTrajectory(executedTrajectory,arena.xBounds,arena.yBounds);

end


function [intersect,intersection,inds] = intersectTrajectory(xCoords,yCoords,xBounds,yBounds)
% identifies the portions of a trajectory that fall within a set of bounds,
% and returns a binary vector that delineates intersections
inds = find(xCoords>=xBounds(1) & xCoords<=xBounds(2) ...
& yCoords>=yBounds(1) & yCoords<=yBounds(2));

if numel(inds)>0
    intersect = 1;
else
    intersect = 0;
end
intersection = zeros(size(xCoords));
intersection(inds) = 1;
end

function [xFace,yFace,exposedFaceID] = getSolidAngle(trial,trialID,xAnchor,yAnchor)
% identifies the region of an obstacle that is visible from an anchor
% point
xFace = [];
yFace = [];
for i=1:4
    exposedFace(i) = intersectTrajectory(xAnchor,yAnchor,...
        trial.obstacle.block(trial.blockIDs(trialID)).region(i).xBounds,...
        trial.obstacle.block(trial.blockIDs(trialID)).region(i).yBounds);
    if exposedFace(i)
        xFace = [xFace,trial.obstacle.block(trial.blockIDs(trialID)).region(i).xBoundary];
        yFace = [yFace,trial.obstacle.block(trial.blockIDs(trialID)).region(i).yBoundary];
    end
end
exposedFaceID = find(exposedFace>0);

end