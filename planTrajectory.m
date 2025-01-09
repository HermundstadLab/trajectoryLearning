function trajectory = planTrajectory(thAnchors,rAnchors,phi,planner)
% PLANTRAJECTORY Plan a trajectory through a set of anchor points.
%   trajectory = PLANTRAJECTORY(thAnchors,rAnchors,phi,planner) uses a 
%   set of ordered anchor points (defined in polar coordinates) and an 
%   initial heading angle phi to plan a curvilinear trajectory through them.  
%   The input 'planner' structure contains parameters that specify the
%   temporal discretization and overall scaling of the trajectory.
%
%   See also: ORDERANCHORS, OPTIMIZETRAJECTORY, EXECUTETRAJECTORY 

% define generative model:
%   dth = angle between successive anchors
%   dr = distance between successive anchors
%   phi = initial heading offset, defined relative to dth-pi/2
%   T = total time to travel between two anchors
velocity  = @(t,amplitude,T) amplitude.*.5*(1-cos(t.*2.*pi./T));
heading   = @(t,phi,dth,T) ((pi-2*phi).*t./T+phi+dth-pi/2);
amplitude = @(dr,phi,T) (dr./T).*(pi-2*phi).*(pi+2*phi).*(3*pi-2*phi)./(4.*pi.^2.*cos(phi));

xtraj = @(t,dr,phi,dth,T) cumsum(velocity(t,amplitude(dr,phi,T),T).*cos(heading(t,phi,dth,T)));
ytraj = @(t,dr,phi,dth,T) cumsum(velocity(t,amplitude(dr,phi,T),T).*sin(heading(t,phi,dth,T)));

% compute scaling
tt = planner.tAxis;
scale = ytraj(tt,1,phi,pi/2,1);
scale = scale(end);

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
    T  = dr./planner.tScale; 
    tt = linspace(0,T,floor(T*planner.nInterp));

    % ensure initial offset is the same as final offset from last segment
    if i>2
        phi = pi-(dth-dth0)-phi0;
    end

    % generate trajectory linking anchors; choose multiple of initial  
    % heading offset that minimizes curvilinear distance
    phiSet = [phi-2*pi,phi,phi+2*pi];
    dist   = nan(size(phiSet));
    [xtmp,ytmp] = deal(nan(size(phiSet,2),numel(tt)));
    for j=1:3
        xtmp(j,:) = x0+xtraj(tt,dr,phiSet(j),dth,T)./scale;
        ytmp(j,:) = y0+ytraj(tt,dr,phiSet(j),dth,T)./scale;
        dist(j)   = sum(dcart(xtmp(j,:),ytmp(j,:)));
    end
    [~,isel] = min(dist);

    % append trajectory
    trajectory.prevAnchor = [trajectory.prevAnchor,(i-1)*ones(1,numel(tt))];
    trajectory.xCoords    = [trajectory.xCoords,   xtmp(isel,:)];
    trajectory.yCoords    = [trajectory.yCoords,   ytmp(isel,:)];
    trajectory.velocity   = [trajectory.velocity,  velocity(tt,amplitude(dr,phiSet(isel),T),T)];
    trajectory.heading    = [trajectory.heading,   heading(tt,phiSet(isel),dth,T)];
    trajectory.amplitude  = [trajectory.amplitude, amplitude(dr,phiSet(isel),T)];
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