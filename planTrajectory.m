function trajectory = planTrajectory(anchors,phi,planner)
% PLANTRAJECTORY Plan a trajectory through a set of anchor points.
%   trajectory = PLANTRAJECTORY(anchors,phi,planner) uses a set of ordered 
%   anchor points (defined in polar coordinates and stored in the structure 
%   'anchors') and an initial heading angle phi to plan a curvilinear 
%   trajectory through them.  The input 'planner' structure contains 
%   parameters that specify the temporal discretization and overall scaling 
%   of the trajectory.
%
%   See also: ORDERANCHORS, GENERATETRAJECTORY, OPTIMIZETRAJECTORY, EXECUTETRAJECTORY 

% initialize trajectory
[x0,y0] = pol2cart(anchors.thCoords(1),anchors.rCoords(1));
dth0    = 0;

% store anchors and initial heading
trajectory.anchors         = anchors;
trajectory.phi             = phi;

% initialize trajectory properties
trajectory.prevAnchor = [];
trajectory.xCoords    = [];
trajectory.yCoords    = [];
trajectory.velocity   = [];
trajectory.heading    = [];
trajectory.timepts    = [];
trajectory.distance   = 0;

t0 = 0;
tAnchors = t0;
for i=2:anchors.N
    
    % compute radial and angular distance between anchors
    [dth,dr] = dpol(anchors.thCoords(i-1:i),anchors.rCoords(i-1:i));

    % scale execution time based on separation between anchors, requiring
    % at least two timepoints per segment
    T  = dr./planner.rScale; 
    tt = linspace(0,T,max(2,floor(T*planner.nInterp)));

    % ensure initial offset is the same as final offset from last segment
    if i>2
        phi = pi-(dth-dth0)-phi0;
    end

    % generate trajectory linking anchors; choose multiple of initial  
    % heading offset that minimizes curvilinear distance
    phiSet = [phi-2*pi,phi,phi+2*pi];
    dist   = nan(size(phiSet));
    [xtmp,ytmp,vtmp,htmp] = deal(nan(size(phiSet,2),numel(tt)));
    for j=1:3
        [xtraj,ytraj,vtmp(j,:),htmp(j,:),dist(j)] = generateTrajectorySegment(tt,dth,dr,phiSet(j),T);
        xtmp(j,:) = x0+xtraj;
        ytmp(j,:) = y0+ytraj;
    end
    [~,isel] = min(dist);

    % append trajectory
    trajectory.prevAnchor = [trajectory.prevAnchor,(i-1)*ones(1,numel(tt))];
    trajectory.xCoords    = [trajectory.xCoords,   xtmp(isel,:)];
    trajectory.yCoords    = [trajectory.yCoords,   ytmp(isel,:)];
    trajectory.velocity   = [trajectory.velocity,  vtmp(isel,:)];
    trajectory.heading    = [trajectory.heading,   htmp(isel,:)];
    trajectory.timepts    = [trajectory.timepts,   t0+tt       ];
    trajectory.distance   = trajectory.distance + dist(isel); 

    % update initial condition
    x0   = xtmp(isel,end);
    y0   = ytmp(isel,end);
    t0   = t0+tt(end);
    dth0 = dth;
    phi0 = phi;

    % update timing of anchor points
    tAnchors = [tAnchors,t0];
end

trajectory.anchors.timepts = tAnchors;
end