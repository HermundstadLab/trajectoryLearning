function trajectory = planTrajectory(thAnchors,rAnchors,phi,planner)
% PLANTRAJECTORY Plan a trajectory through a set of anchor points.
%   trajectory = PLANTRAJECTORY(thAnchors,rAnchors,phi,planner) uses a 
%   set of ordered anchor points (defined in polar coordinates) and an 
%   initial heading angle phi to plan a curvilinear trajectory through them.  
%   The input 'planner' structure contains parameters that specify the
%   temporal discretization and overall scaling of the trajectory.
%
%   See also: ORDERANCHORS, GENERATETRAJECTORY, OPTIMIZETRAJECTORY, EXECUTETRAJECTORY 

% initialize trajectory
[x0,y0] = pol2cart(thAnchors(1),rAnchors(1));
dth0    = 0;

trajectory.prevAnchor = [];
trajectory.xCoords    = [];
trajectory.yCoords    = [];
trajectory.velocity   = [];
trajectory.heading    = [];
trajectory.amplitude  = [];
trajectory.offset     = [];

t0 = 0;
for i=2:numel(rAnchors)
    
    % compute radial and angular distance between anchors
    [dth,dr] = dpol(thAnchors(i-1:i),rAnchors(i-1:i));
    T  = dr./planner.rScale; 
    tt = linspace(0,T,floor(T*planner.nInterp));

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
        [xtraj,ytraj,vtmp(j,:),htmp(j,:)] = generateTrajectorySegment(tt,dth,dr,phiSet(j),T);
        xtmp(j,:) = x0+xtraj./planner.tScale;
        ytmp(j,:) = y0+ytraj./planner.tScale;
        dist(j)   = sum(dcart(xtmp(j,:),ytmp(j,:)));
    end
    [~,isel] = min(dist);

    % append trajectory
    trajectory.prevAnchor = [trajectory.prevAnchor,(i-1)*ones(1,numel(tt))];
    trajectory.xCoords    = [trajectory.xCoords,   xtmp(isel,:)];
    trajectory.yCoords    = [trajectory.yCoords,   ytmp(isel,:)];
    trajectory.velocity   = [trajectory.velocity,  vtmp(isel,:)];
    trajectory.heading    = [trajectory.heading,   htmp(isel,:)];
    trajectory.offset     = [trajectory.offset,    phiSet(isel)];

    % update initial condition
    x0   = xtmp(isel,end);
    y0   = ytmp(isel,end);
    t0   = t0+tt(end);
    dth0 = dth;
    phi0 = phi;
end

trajectory.thAnchors = thAnchors;
trajectory.rAnchors  = rAnchors;

end