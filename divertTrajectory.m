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

% extract agent's view of obstacle
agentView = trial.obstacle.agent;
intersectionType = 'inside';

% extract block ID
blockID = trial.blockIDs(trialID);

% determine which portions of the trajectory were affected by obstacle ('obstacleVec')
[~,obstacleVec,~] = intersectTrajectory(trajectory.xCoords,trajectory.yCoords,...
    agentView.xBounds(blockID,:),agentView.yBounds(blockID,:),intersectionType);

% find timepoint just before agent enters obstacle
entrances = find(diff(obstacleVec)>0);     

% specify entry angles along boundaries, indexed by:
%   boundary ID: 1:bottom; 2:right; 3:top; 4:left (rows of array)
%   orientation: 1:CW; 2:CCW (columns of array)
boundaryAngles = [[0, pi    ,2*pi  ];...
   [    pi/2, 3*pi/2,5*pi/2];...
   [      pi, 2*pi  , -pi  ];...
   [  3*pi/2, pi/2  ,-pi/2 ]];

% initialize fields for updating trajectories and boundary segments
trajFields     = {'prevAnchor','xCoords','yCoords','velocity','heading'};
anchorFields   = {'thCoords','rCoords','thTol','rTol'};
boundaryFields = {'xCoords','yCoords','heading','corners'};

% initialize executed trajectory:
originalTrajectory = trajectory;
executedTrajectory = planEmptyTrajectory(trajFields);

% store first trajectory segment from home port to first boundary encounter
inds = 1:entrances(1);
executedTrajectory = expandTrajectory(executedTrajectory,originalTrajectory,trajFields,inds);
originalTrajectory = trimTrajectory(originalTrajectory,trajFields,inds);
obstacleVec(inds) = [];

while numel(obstacleVec)>0 && obstacleVec(1)>0
    if numel(obstacleVec)==0 || obstacleVec(1)==0 
        break
    end
    
    % determine exit point from boundary
    trajectoryInds.boundaryExit = min([numel(obstacleVec),find(diff(obstacleVec)<0,1,'first')+1]);
    trajectoryInds.boundaryIntersection = 1:trajectoryInds.boundaryExit;

    % extract segment of trajectory that intersected with boundary
    trajectorySegment.boundaryIntersection = extractTrajectorySegment(originalTrajectory,trajectoryInds.boundaryIntersection,trajFields);
    boundarySegment = extractBoundarySegment(trajectorySegment.boundaryIntersection,agentView,boundaryAngles,blockID);

    % extract next anchor point after exiting boundary
    [nextAnchor,trajectoryInds.nextAnchor] = extractNextAnchor(originalTrajectory,agentView,blockID,intersectionType);

    % extract face of boundary that is exposed to next anchor point
    [exposedFace,boundaryInds.exposedFace] = getExposedFace(nextAnchor,boundarySegment,agentView,blockID);

    % determine how to divert trajectory around obstacle

    %%%%%%%%%%%%%%%%%%%%%%%% DIVERSION SCENARIOS %%%%%%%%%%%%%%%%%%%%%%%%%%
    % OPTION 1: the next anchor is reachable                              %
    %   OPTION 1A: the next anchor is already visible to the agent when   % 
    %   it encounters the boundary                                        %
    %       response: agent executes the entire diverted trajectory until %
    %                 it reaches original exit point from the boundary    %
    %   OPTION 1B: the next anchor is not visible to the agent when it    %
    %   encounters the boundary                                           %
    %       response: agent traverses the boundary trajectory until the   %
    %       next anchor point becomes visible, at which point it peels    %
    %       away from the boundary                                        %
    %                                                                     %
    % OPTION 2: the next anchor is unreachable (i.e. it is within the     %
    % obstacle) or it in not along the faces of the obstacle along which  %
    % the trajectory would be diverted                                    %
    %       response: the agent must alter the boundary trajectory        %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % OPTION 1: if next anchor is reachable
    if nextAnchor.nextAnchorID-nextAnchor.prevAnchorID==1 && numel(boundaryInds.exposedFace)>0 

        % OPTION 1A: if agent is already on exposed boundary face
        if ismember(boundarySegment.entryFaceID,exposedFace.exposedFaceID) 

            % use exit point from boundary as anchor point, and generate
            % a trajectory segment along boundary
            [boundaryTrajectory,boundaryInds] = generateBoundaryTrajectory(boundarySegment,nextAnchor,boundaryInds,planner);
      
            % add boundary trajectory to existing trajectory
            executedTrajectory = expandTrajectory(executedTrajectory,boundaryTrajectory,trajFields);

            % add remaining portion of original trajectory until next anchor
            trajectoryInds.OpenField = trajectoryInds.boundaryExit+1:trajectoryInds.nextAnchor; 
            executedTrajectory = expandTrajectory(executedTrajectory,originalTrajectory,trajFields,trajectoryInds.OpenField);

            % trim original trajectory
            trajectoryInds.divertedSegment = [trajectoryInds.boundaryIntersection,trajectoryInds.OpenField];
            originalTrajectory = trimTrajectory(originalTrajectory,trajFields,trajectoryInds.divertedSegment);
            obstacleVec(trajectoryInds.divertedSegment) = [];

        % OPTION 1B: if agent is not on exposed boundary face
        else
            % split boundary segment at point where agent can see next anchor
            indSplit = boundaryInds.exposedFace(1);
            indEnd   = numel(boundarySegment.xCoords);

            if indSplit==numel(boundarySegment.xCoords)
                % extract full boundary segment
                boundarySegment = extractBoundarySegment(trajectorySegment.boundaryIntersection,agentView,boundaryAngles,blockID,1);
                indEnd   = numel(boundarySegment.xCoords);
            end
            [boundarySegment1,boundarySegment2] = splitTrajectory(boundarySegment,boundaryFields,indSplit,indEnd);

            % generate boundary trajectory
            [boundaryTrajectory,boundaryInds] = generateBoundaryTrajectory(boundarySegment1,nextAnchor,boundaryInds,planner);

            % add boundary trajectory to existing trajectory
            executedTrajectory = expandTrajectory(executedTrajectory,boundaryTrajectory,trajFields);

            % generate open field trajectory to next anchor point
            openFieldTrajectory = generateOpenFieldTrajectory(originalTrajectory,boundarySegment2,nextAnchor,trajectoryInds,planner);

            % add open field trajectory to existing trajectory
            executedTrajectory = expandTrajectory(executedTrajectory,openFieldTrajectory,trajFields);

            % trim original trajectory
            trajectoryInds.OpenField = trajectoryInds.boundaryExit+1:trajectoryInds.nextAnchor;
            trajectoryInds.divertedSegment = [trajectoryInds.boundaryIntersection,trajectoryInds.OpenField];
            originalTrajectory = trimTrajectory(originalTrajectory,trajFields,trajectoryInds.divertedSegment);
            obstacleVec(trajectoryInds.divertedSegment) = [];

        end

    % OPTION 2: if next anchor is unreachable, proceed to subsequent anchor
    elseif nextAnchor.nextAnchorID-nextAnchor.prevAnchorID>1 || numel(boundaryInds.exposedFace)==0 

        % extract full boundary segment
        boundarySegment = extractBoundarySegment(trajectorySegment.boundaryIntersection,agentView,boundaryAngles,blockID,1);

        % find point along boundary segment that first intersects exposed face
        [~,indSplit] = min(abs(boundarySegment.yCoords'-exposedFace.yCoords([1,end]))...
            + abs(boundarySegment.xCoords'-exposedFace.xCoords([1,end])),[],1);
        indEnd   = numel(boundarySegment.xCoords);

        % split boundary segment at point where agent can see next anchor 
        [boundarySegment1,boundarySegment2] = splitTrajectory(boundarySegment,boundaryFields,min(indSplit),indEnd);

        % generate boundary trajectory
        [boundaryTrajectory,boundaryInds] = generateBoundaryTrajectory(boundarySegment1,nextAnchor,boundaryInds,planner);

        % add boundary trajectory to existing trajectory
        executedTrajectory = expandTrajectory(executedTrajectory,boundaryTrajectory,trajFields);

        % generate open field trajectory to next anchor point
        openFieldTrajectory = generateOpenFieldTrajectory(originalTrajectory,boundarySegment2,nextAnchor,trajectoryInds,planner);

        % add open field trajectory to existing trajectory
        executedTrajectory = expandTrajectory(executedTrajectory,openFieldTrajectory,trajFields);

        % trim original trajectory
        trajectoryInds.OpenField = trajectoryInds.boundaryExit+1:trajectoryInds.nextAnchor;
        trajectoryInds.divertedSegment = [trajectoryInds.boundaryIntersection,trajectoryInds.OpenField];
        originalTrajectory = trimTrajectory(originalTrajectory,trajFields,trajectoryInds.divertedSegment);
        obstacleVec(trajectoryInds.divertedSegment) = [];

    else
        error('anchor IDs do not match')   
    end

    % add remaining trajectory until next obstacle intersection
    inds = 1:find(diff(obstacleVec)>0,1,'first');
    executedTrajectory = expandTrajectory(executedTrajectory,originalTrajectory,trajFields,inds);

    % trim original trajectory
    originalTrajectory = trimTrajectory(originalTrajectory,trajFields,inds);
    obstacleVec(inds) = [];

end

% append remaining trajectory
executedTrajectory = expandTrajectory(executedTrajectory,originalTrajectory,trajFields);

% compute & store curvilinear distance along trajectory:
executedTrajectory.distance = sum(dcart(executedTrajectory.xCoords,executedTrajectory.yCoords),'omitnan');

% update anchors that were executed:
executedTrajectory = updateAnchors(executedTrajectory,originalTrajectory,anchorFields);

% store initial heading and boundary flag:
executedTrajectory.delta        = trajectory.delta;
executedTrajectory.boundaryFlag = trajectory.boundaryFlag;
x = 5;

end

function traj = updateAnchors(traj,origTraj,anchorFields)
% extract unique anchor IDs
executedAnchors = unique(traj.prevAnchor);
executedAnchors = [executedAnchors,max(executedAnchors)+1];

anchors = planEmptyTrajectory(anchorFields);
anchors = expandTrajectory(anchors,origTraj.anchors,anchorFields,executedAnchors);
anchors.N = numel(anchors.thCoords);

% update IDs of unique anchors
anchorsUnique = unique(traj.prevAnchor);
currentAnchor = 1;
for i=1:numel(anchorsUnique)
    ii = find(traj.prevAnchor==anchorsUnique(i));
    traj.prevAnchor(ii) = currentAnchor;
    currentAnchor = currentAnchor+1;
end

% append anchors
traj.anchors = anchors;

if any(diff(traj.prevAnchor)>1) || numel(unique(traj.prevAnchor))~=(traj.anchors.N-1)
    error('mismatched anchors')
end
end

function [exposedFace,indExposed] = getExposedFace(anchor,traj,agentView,blockID)
% extract the obstacle faces that are exposed to the anchor point
[exposedFace.xCoords,exposedFace.yCoords,exposedFace.exposedFaceID] = getSolidAngle(agentView,blockID,anchor.xCoords,anchor.yCoords);

% find the point along the boundary segment that first intersects the
% exposed faces of the obstacle (the first entry in 'indExposed')
[~,indExposed,~] = intersect([traj.xCoords',traj.yCoords'],...
    [exposedFace.xCoords',exposedFace.yCoords'],'rows','stable');

end

function [anchor,index] = extractNextAnchor(traj,agentView,blockID,intersectionType)

% extract first anchor point after trajectory passes through obstacle
prevAnchorID = traj.prevAnchor(1);
nextAnchorID = prevAnchorID+1;
[xAnchor,yAnchor] = pol2cart(traj.anchors.thCoords(nextAnchorID),...
    traj.anchors.rCoords(nextAnchorID));

% if next anchor is within the obstacle, select the subsequent anchor
while intersectTrajectory(xAnchor,yAnchor,agentView.xBounds(blockID,:),agentView.yBounds(blockID,:),intersectionType)
    nextAnchorID = nextAnchorID+1;
    [xAnchor,yAnchor] = pol2cart(traj.anchors.thCoords(nextAnchorID),...
        traj.anchors.rCoords(nextAnchorID));
end
anchor.xCoords  = xAnchor;
anchor.yCoords  = yAnchor;
anchor.thCoords = traj.anchors.thCoords(nextAnchorID);
anchor.rCoords  = traj.anchors.rCoords( nextAnchorID);
anchor.prevAnchorID = prevAnchorID;
anchor.nextAnchorID = nextAnchorID;

if nextAnchorID>max(traj.prevAnchor)
    index = numel(traj.prevAnchor);
else
    index = find(traj.prevAnchor==nextAnchorID,1,'first');
end
end

function boundarySeg = extractBoundarySegment(traj,agentView,boundaryAngles,blockID,extractFullBoundary)
if nargin<5
    extractFullBoundary = 0;
end

% determine which obstacle boundary the trajectory entered:
% 1:bottom; 2:right; 3:top; 4:left
[~,indStart] = min( abs(traj.xCoords(1)-agentView.xBoundary(blockID,:)) ...
    + abs(traj.yCoords(1)-agentView.yBoundary(blockID,:)) );
boundaryID   = find(histcounts(indStart,agentView.indCorners(blockID,:)));

% determine the travel direction that minimizes the change in heading
[~,orientation] = min(abs(wrapTo2Pi(traj.heading(1)) - boundaryAngles(boundaryID,:)));

% extract oriented segment of boundary along which trajectory will be diverted
xBoundary  = circshift(agentView.xBoundary(blockID,:),numel(agentView.xBoundary(blockID,:))-indStart+1);
yBoundary  = circshift(agentView.yBoundary(blockID,:),numel(agentView.yBoundary(blockID,:))-indStart+1);
heading    = circshift(agentView.heading(  blockID,:),numel(agentView.heading(  blockID,:))-indStart+1);
corners    = circshift(agentView.corners(  blockID,:),numel(agentView.corners(  blockID,:))-indStart+1);
if mod(orientation,2)==0
    xBoundary = fliplr(xBoundary);
    yBoundary = fliplr(yBoundary);
    heading   = fliplr(heading);
    heading   = wrapTo2Pi(heading+pi);
    corners   = fliplr(corners);
end

if extractFullBoundary
    indEnd = numel(xBoundary);
else
    [~,indEnd] = min(abs(traj.xCoords(end)-xBoundary) + abs(traj.yCoords(end)-yBoundary));
end
boundarySeg.xCoords = xBoundary(1:indEnd);
boundarySeg.yCoords = yBoundary(1:indEnd);
boundarySeg.heading = heading(  1:indEnd);
boundarySeg.corners = corners(  1:indEnd);
boundarySeg.entryFaceID = boundaryID;

end

function trajSeg = extractTrajectorySegment(traj,indsSegment,trajFields)
for i=1:numel(trajFields)
    trajSeg.(trajFields{i}) = traj.(trajFields{i})(indsSegment);
end
end

function openFieldTrajectory = generateOpenFieldTrajectory(traj,boundarySegment,nextAnchor,trajectoryInds,planner)
% once trajectory reaches the first exposed corner of the obstacle, use the
% corner as an anchor, and generate a trajectory to the next anchor:
[openFieldAnchors.thCoords,openFieldAnchors.rCoords] = cart2pol(boundarySegment.xCoords(1),boundarySegment.yCoords(1));
openFieldAnchors.thCoords = [openFieldAnchors.thCoords,traj.anchors.thCoords(nextAnchor.nextAnchorID)];
openFieldAnchors.rCoords  = [openFieldAnchors.rCoords, traj.anchors.rCoords( nextAnchor.nextAnchorID)];
openFieldAnchors.N = numel(openFieldAnchors.thCoords);

% choose initial heading s.t. agent initially follows obstacle boundary 
% before peeling away; if this generates a large discontinuity in heading
% at next anchor point, choose direct path. 
initialHeading = boundarySegment.heading(1);

[dth, ~] = dpol(openFieldAnchors.thCoords,openFieldAnchors.rCoords);
delta    = initialHeading-dth+pi/2;
deltaSet = [delta,pi/2];

% if the next anchor point is the final one (i.e., the home port), peel
% away from the boundary (i.e., choose first of allowed initial headings)
if nextAnchor.nextAnchorID==max(traj.prevAnchor)+1
    delta = deltaSet(1);

% otherwise, choose the initial heading that minimizes the difference
% with the subsequent heading
else
    angleVec = [-2*pi,0,2*pi]';
    nextAnchorHeading = traj.heading(trajectoryInds.nextAnchor);
    finalHeading = pi/2+dth-deltaSet;
    [~,indMin] = min(min(abs(finalHeading-(nextAnchorHeading-angleVec)),[],1));
    delta = deltaSet(indMin);
end

openFieldTrajectory = planTrajectory(openFieldAnchors,delta,planner,1);
openFieldTrajectory.prevAnchor = (nextAnchor.prevAnchorID).*ones(1,numel(openFieldTrajectory.xCoords));

end

function [boundaryTrajectory,boundaryInds] = generateBoundaryTrajectory(boundarySegment,nextAnchor,boundaryInds,planner)
boundaryInds.exitPoint = numel(boundarySegment.xCoords);
boundaryInds.boundary  = 1:numel(boundarySegment.xCoords);
boundaryInds.corners   = find(boundarySegment.corners);
boundaryInds.anchors   = [1,boundaryInds.corners,boundaryInds.exitPoint];

[boundaryAnchors.thCoords,boundaryAnchors.rCoords] = cart2pol(boundarySegment.xCoords(boundaryInds.anchors),boundarySegment.yCoords(boundaryInds.anchors));
boundaryAnchors.N  = numel(boundaryAnchors.thCoords);
boundaryTrajectory = planTrajectory(boundaryAnchors,pi/2,planner,1);
boundaryTrajectory.prevAnchor = (nextAnchor.prevAnchorID).*ones(1,numel(boundaryTrajectory.xCoords));
end

function traj = expandTrajectory(traj,segment,trajFields,indsSegment)
if nargin<4
    indsSegment = 1:numel(segment.(trajFields{1}));
end
for i=1:numel(trajFields)
    traj.(trajFields{i}) = ...
        [traj.(trajFields{i}),...
        segment.(trajFields{i})(indsSegment)];
end
end

function [traj1,traj2] = splitTrajectory(traj,trajFields,indSplit,indEnd)
for i=1:numel(trajFields)
    traj1.(trajFields{i}) = traj.(trajFields{i})(1:indSplit);
    traj2.(trajFields{i}) = traj.(trajFields{i})(indSplit+1:indEnd);
end
end

function traj = trimTrajectory(traj,trajFields,indsTrim)
for i=1:numel(trajFields)
    traj.(trajFields{i})(indsTrim) = [];
end
end

function traj = planEmptyTrajectory(trajFields)
for i=1:numel(trajFields)
    traj.(trajFields{i}) = [];
end
end

function [xFace,yFace,exposedFaceID] = getSolidAngle(agentView,blockID,xAnchor,yAnchor)
% identifies the region of an obstacle that is visible from an anchor point
xFace = [];
yFace = [];
for i=1:4
    exposedFace(i) = intersectTrajectory(xAnchor,yAnchor,...
        agentView.block(blockID).region(i).xBounds,...
        agentView.block(blockID).region(i).yBounds,'inside');
    if exposedFace(i)
        xFace = [xFace,agentView.block(blockID).region(i).xBoundary];
        yFace = [yFace,agentView.block(blockID).region(i).yBoundary];
    end
end
exposedFaceID = find(exposedFace>0);

end