function trajectory = planTrajectory(anchors,delta,planner,boundaryFlag)
% PLANTRAJECTORY Plan a trajectory through a set of anchor points.
%   trajectory = PLANTRAJECTORY(anchors,delta,planner) uses a set of ordered 
%   anchor points (defined in polar coordinates and stored in the structure 
%   'anchors') and an initial heading angle delta to plan a curvilinear 
%   trajectory through them.  The input 'planner' structure contains 
%   parameters that specify the temporal discretization and overall scaling 
%   of the trajectory.
%
%   See also: ORDERANCHORS, GENERATETRAJECTORY, OPTIMIZETRAJECTORY, EXECUTETRAJECTORY 

if nargin<4
    boundaryFlag = false;
end

% initialize trajectory
[x0,y0] = pol2cart(anchors.thCoords(1),anchors.rCoords(1));
dth0    = 0;

% store anchors and initial heading
trajectory.anchors         = anchors;
trajectory.delta           = delta;

% initialize trajectory properties
trajectory.prevAnchor = [];
trajectory.xCoords    = [];
trajectory.yCoords    = [];
trajectory.velocity   = [];
trajectory.heading    = [];
trajectory.distance   = 0;

trajectory.boundaryFlag = boundaryFlag;

for i=2:anchors.N
    
    % compute radial and angular distance between anchors
    [dth,dr] = dpol(anchors.thCoords(i-1:i),anchors.rCoords(i-1:i));

    % scale execution time based on separation between anchors, requiring
    % at least two timepoints per segment
    T  = dr./planner.rScale; 
    tt = 0:planner.dt:T;

    % if generating last segment of trajectory, append final timepoint 
    if i==anchors.N && tt(end)<T
       tt = [tt,T]; 
    end

    % for any trajectory besides a boundary trajectory, ensure the initial 
    % offset is the same as final offset from last segment
    if i>2 && ~boundaryFlag
        delta = wrapToPi(pi-(dth-dth0)-delta0);
    end

    % generate trajectory linking anchors; choose multiple of initial  
    % heading offset that minimizes curvilinear distance
    deltaSet = [delta-2*pi,delta,delta+2*pi];
    dist   = nan(size(deltaSet));
    [xtmp,ytmp,vtmp,htmp] = deal(nan(size(deltaSet,2),numel(tt)));
    for j=1:3
        [xtraj,ytraj,vtmp(j,:),htmp(j,:),dist(j)] = generateTrajectorySegment(tt,dth,dr,deltaSet(j),T);
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
    trajectory.distance   = trajectory.distance + dist(isel); 

    % update initial condition
    [x0,y0] = pol2cart(anchors.thCoords(i),anchors.rCoords(i));
    dth0    = dth;
    delta0  = delta;
    
end

end