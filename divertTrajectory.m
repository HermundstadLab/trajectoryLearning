function executedTrajectory = divertTrajectory(trajectory,trial,planner,trialID)
% DIVERTTRAJECTORY Divert trajectory around obstacles.
%
%   executedTrajectory = DIVERTTRAJECTORY(trajectory,trial,planner,trialID)
%   takes as input a planned trajectory, and executes a version of the
%   trajectory that does not intersect obstacles. The input structure 
%   'trial' contains trial-specific properties of the obstacle boundaries;
%   the input structure 'planner' contains information about the agent's 
%   heading and velocity around obstacles. 
%
%   See also: PLANTRAJECTORY, BOUNDTRAJECTORY, EXECUTETRAJECTORY


% determine which portions of the trajectory were affected by obstacle ('obstacleVec')
[~,obstacleVec,~] = intersectTrajectory(trajectory.xCoords,trajectory.yCoords,...
    trial.obstacle.xBounds(trial.blockIDs(trialID),:),...
    trial.obstacle.yBounds(trial.blockIDs(trialID),:));

% find timepoints when trajectory enters and exits obstacle
entrances = find(diff(obstacleVec)>0);
exits     = find(diff(obstacleVec)<0)+1;

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
    while intersectTrajectory(xAnchor,yAnchor,trial.obstacle.xBounds(blockID,:),trial.obstacle.yBounds(blockID,:))
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

    % extract index of next anchor point
    nextAnchorIndex = find(trajectory.prevAnchor==nextAnchorID,1,'first');

    % if the agent is already on the exposed face, or if the exposed face 
    % is not contained in the set of boundaries that the agent traverses
    % along its diverted trajectory, assume the agent executes the entire
    % diverted trajectory until it reaches its original exit point from
    % the obstacle  
    if ismember(boundaryID,exposedFaceID) || numel(indExposed)==0
        indsTrajBound = 1:numel(xBoundary);
        trajDivertedBoundary.prevAnchor = prevAnchorID.*ones(size(indsTrajBound));
        trajDivertedBoundary.xCoords    = xBoundary;
        trajDivertedBoundary.yCoords    = yBoundary;
        trajDivertedBoundary.velocity   = planner.boundaryVelocity.*ones(size(indsTrajBound));
        trajDivertedBoundary.heading    = heading;

        indsTrajOpen = exits(i)+1:nextAnchorIndex;
        trajDivertedOpen.prevAnchor     = prevAnchorID.*ones(size(indsTrajOpen));
        trajDivertedOpen.xCoords        = trajectory.xCoords(     indsTrajOpen);
        trajDivertedOpen.yCoords        = trajectory.yCoords(     indsTrajOpen);
        trajDivertedOpen.velocity       = trajectory.velocity(    indsTrajOpen);
        trajDivertedOpen.heading        = trajectory.heading(     indsTrajOpen);

    % otherwise, assume the agent travels around obstacle boundary until it
    % can peel away toward next anchor point
    else

        % generate the first portion of the diverted trajectory by moving at 
        % an approximately constant speed around the unexposed portion of the 
        % obstacle:
        indsTrajBound = 1:indExposed(1)-1;
        trajDivertedBoundary.prevAnchor = prevAnchorID.*ones(1,max(1,numel(indsTrajBound)));
        trajDivertedBoundary.xCoords    = xBoundary(indsTrajBound);
        trajDivertedBoundary.yCoords    = yBoundary(indsTrajBound);
        trajDivertedBoundary.velocity   = planner.boundaryVelocity.*ones(size(indsTrajBound));
        trajDivertedBoundary.heading    = heading(indsTrajBound);

        % once trajectory reaches the first exposed corner of the obstacle,
        % use the corner as an anchor point, and generate a standard trajectory 
        % segment to the next (i.e., the original) anchor point:
        [anchors.thCoords,anchors.rCoords] = cart2pol(xBoundary(indExposed(1)),yBoundary(indExposed(1)));
        anchors.thCoords = [anchors.thCoords,trajectory.anchors.thCoords(nextAnchorID)];
        anchors.rCoords  = [anchors.rCoords, trajectory.anchors.rCoords( nextAnchorID)];
        anchors.N = numel(anchors.thCoords);
    
        % choose initial heading such that agent initially follows obstacle 
        % boundary before peeling away; if this generates a large discontinuity
        % in heading at next anchor point, choose the negative of this heading. 
        if numel(indsTrajBound)<1
            nextBoundaryHeading = heading(1);
        else
            nextBoundaryHeading = heading(indsTrajBound(end)+1);
        end
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
       
        trajDivertedOpen            = planTrajectory(anchors,phi,planner);
        trajDivertedOpen.prevAnchor = trajDivertedOpen.prevAnchor+prevAnchorID-1;
    end

    % store diverted trajectory segments to be inserted into planned trajectory:
    boundarySegments{i} = trajDivertedBoundary;
    openSegments{i}     = trajDivertedOpen;

end

% insert diverted trajectory segments into planned trajectory
trajFields = {'prevAnchor','xCoords','yCoords','velocity','heading'};
if numel(entrances)>0

    % remove any anchor points that fell within the obstacle:
    [anchor_xCoords,anchor_yCoords] = pol2cart(trajectory.anchors.thCoords,trajectory.anchors.rCoords);
    [~,anchorVec,~] = intersectTrajectory(anchor_xCoords,anchor_yCoords,...
        trial.obstacle.xBounds(trial.blockIDs(trialID),:),...
        trial.obstacle.yBounds(trial.blockIDs(trialID),:));
    irem = find(anchorVec);
    anchorFields = {'thCoords','rCoords','thTol','rTol'};
    if numel(irem)>0
        for i=1:numel(anchorFields)
            trajectory.anchors.(anchorFields{i})(irem) = [];
        end
    end
    trajectory.anchors.N = numel(trajectory.anchors.thCoords);

    % store anchors and initial heading:
    executedTrajectory.anchors  = trajectory.anchors;
    executedTrajectory.phi      = trajectory.phi;


    % store trajectory segment until first intersection with obstacle;
    % update and store timepoints separately
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
    
    % compute & store curvilinear distance along trajectory:
    executedTrajectory.distance = dcart(executedTrajectory.xCoords,executedTrajectory.yCoords);

else
    executedTrajectory = trajectory;
end

end

function [xFace,yFace,exposedFaceID] = getSolidAngle(trial,trialID,xAnchor,yAnchor)
% identifies the region of an obstacle that is visible from an anchor point
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