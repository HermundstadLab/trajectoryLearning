function trajectory = optimizeTrajectory(anchors,belief,planner)
% OPTIMIZETRAJECTORY Plan and optimize a trajectory.
%
%   trajectory = OPTIMIZETRAJECTORY(anchors,belief,planner) uses a set of
%   input anchor points (defined in polar coordinates and stored in the
%   structure 'anchors') to plan an optimized curvilinear trajectory 
%   through them. This function first determines the optimal ordering of
%   anchors that minimizes the pairwise distance between them. Given this 
%   ordering, it then determines the initial heading angle and precise 
%   anchor locations (within some tolerance of the original locations, as 
%   specified by the 'belief' and 'planner' structures) that jointly 
%   minimize the total curvilinear distance along the trajectory.
%
%   See also: ORDERANCHORS, PLANTRAJECTORY


%---------------------- check for boundary run ---------------------------%
% if majority of anchors lie along the boundary, plan boundary run
[boundaryAnchorFlag,boundaryAnchors] = checkBoundary(anchors,belief);
if numel(boundaryAnchorFlag)>1 & mean(boundaryAnchorFlag)>0.5

    newAnchors = updateAnchorOrder(boundaryAnchors,boundaryAnchorFlag,planner);
    trajectory = planTrajectory(newAnchors,pi/2,planner,1);

% otherwise, execute normal run
else

    %---------------------- optimize anchor order ------------------------%
    % determine optimal ordering of anchors
    anchorsOrdered = orderAnchors(anchors,planner);
    
    
    %-------------------- optimize anchor locations ----------------------%
    % determine specific placement of anchors, within some tolerance of 
    % their original locations
    nAnchors = anchorsOrdered.N;
    
    if planner.scaleTol
        dthLB = anchorsOrdered.thTol;                   % angular shift, lower bound 
        drLB  = anchorsOrdered.rTol;                    % radial shift,  lower bound 
    else
        dthLB = planner.thTol_shift.*ones(1,nAnchors);  % angular shift, lower bound 
        drLB  = planner.rTol_shift.*ones(1,nAnchors);   % radial shift,  lower bound 
    end
    dthUB = dthLB;                                      % angular shift, upper bound 
    drUB  = drLB;                                       % radial shift,  upper bound 
    
    [dth,~] = dpol(anchorsOrdered.thCoords(1:2),anchorsOrdered.rCoords(1:2));
    deltaMin = max(0,pi/2-dth);
    deltaMax = min(pi,3*pi/2-dth);

    % define bounds for optimization (defined by the maximum of the lower bound
    % and the minimum boundaries of the arena, and similarly by the minimum of
    % the upper bound and the maximum boundaries of the arena):
    LB = max([[belief.thMin.*ones(1,nAnchors),[belief.rMin,belief.rMinAnchors.*ones(1,nAnchors-2),belief.rMin],deltaMin];...
              [anchorsOrdered.thCoords-dthLB, anchorsOrdered.rCoords-drLB,                                     deltaMin]],[],1);
    UB = min([[belief.thMax.*ones(1,nAnchors),belief.rMax.*ones(1,nAnchors),deltaMax];...
              [anchorsOrdered.thCoords+dthUB, anchorsOrdered.rCoords+drUB,  deltaMax]],[],1);
    
    % define initial condition (add small uniform noise to original anchor locations)
    delta = (deltaMax-deltaMin)*rand()+deltaMin;                     % initial guess for first heading angle
    p0  = [anchorsOrdered.thCoords + anchorsOrdered.thTol.*rand(1,nAnchors)-anchorsOrdered.thTol/2,...
           anchorsOrdered.rCoords  + anchorsOrdered.rTol.*rand( 1,nAnchors)-anchorsOrdered.rTol/2,...
           delta];
    e0  = errFnc([anchorsOrdered.thCoords,anchorsOrdered.rCoords,delta]);
    
    % optimize anchor placement
    options = optimoptions('fmincon','display','off','Algorithm','sqp');
    [pmin,emin] = fmincon(@errFnc,p0,[],[],[],[],LB,UB,[],options);
    
    % accept optimization if final curvilinear distance is lower than initial
    if emin<e0
        anchorsOpt.thCoords = pmin(1:nAnchors);
        anchorsOpt.rCoords  = pmin(nAnchors+1:end-1);
        anchorsOpt.thTol    = anchorsOrdered.thTol;
        anchorsOpt.rTol     = anchorsOrdered.rTol;
        anchorsOpt.N        = nAnchors;
        deltaOpt              = pmin(end);
    
        trajectory = planTrajectory(anchorsOpt,deltaOpt,planner);
    else
        trajectory = planTrajectory(anchorsOrdered,delta,planner);
    end
    

end

    function err = errFnc(params)
        anch.N = (numel(params)-1)/2;
        anch.thCoords = params(1:anch.N); 
        anch.rCoords  = params(anch.N+1:2*anch.N); 
        traj = planTrajectory(anch,params(end),planner);
        err  = traj.distance;
    end

% determine whether trajectory would hit arena boundaries
arenaHit = intersectTrajectory(trajectory.xCoords,trajectory.yCoords,...
    planner.boundary.xBounds,planner.boundary.yBounds,'outside');

% if trajectory would hit arena boundary, bound trajectory along it
if arenaHit
    trajectory = boundTrajectory(trajectory,planner.boundary.xBounds,planner.boundary.yBounds);
end


end

function anchors = updateAnchorOrder(boundaryAnchors,boundaryAnchorFlag,planner)
% Combines boundary anchors with anchors positioned at corners of arena, 
% and orders the set of anchors along the boundary of the arena

thCoords = [planner.boundary.anchors.thCoords(2:end-1),boundaryAnchors.thCoords(boundaryAnchorFlag>0)];
rCoords  = [planner.boundary.anchors.rCoords( 2:end-1),boundaryAnchors.rCoords( boundaryAnchorFlag>0)];

% remove redundant anchors
anchors_unique  = unique([thCoords',rCoords'],'rows');
thCoords = anchors_unique(:,1)';
rCoords  = anchors_unique(:,2)';

% group anchors based on their angle
i1 = find(thCoords==0);
i2 = find(thCoords>0 & thCoords<pi);
i3 = find(thCoords==pi);

% sort with each group
[~,isort1] = sort(rCoords( i1),'ascend');
[~,isort2] = sort(thCoords(i2),'ascend');
[~,isort3] = sort(rCoords( i3),'descend');

% append sorted anchors
anchors.thCoords = [0,thCoords(i1(isort1)),thCoords(i2(isort2)),thCoords(i3(isort3)),0];
anchors.rCoords  = [0, rCoords(i1(isort1)), rCoords(i2(isort2)), rCoords(i3(isort3)),0];
anchors.N        = numel(anchors.thCoords);

end

function [boundaryAnchors,anchors] = checkBoundary(anchors,belief)
% Returns a binary vector whose nonzero entries flag anchors within a given
% tolerance of the arena boundary

dth = abs(anchors.thCoords-belief.thBoundary');
dr  = abs(anchors.rCoords -belief.rBoundary' );
[~,ind] = min(dr+dth);

dist = nan(1,anchors.N);
boundaryAnchors = zeros(1,anchors.N);
for i=1:anchors.N
    [~,dist(i)] = dpol([anchors.thCoords(i),belief.thBoundary(ind(i))]./diff(belief.thBounds),...
        [anchors.rCoords(i),belief.rBoundary(ind(i))]./diff(belief.rBounds));

    if dist(i)<belief.boundaryTol
        boundaryAnchors(i)  = 1;
        anchors.thCoords(i) = belief.thBoundary(ind(i));
        anchors.rCoords(i)  = belief.rBoundary( ind(i));
    end
end

end