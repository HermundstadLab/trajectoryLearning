function trajectory = planTrajectory(thAnchors,rAnchors,Delta,planner)
% PLANTRAJECTORY Plan a trajectory through a set of anchor points.
%   trajectory = PLANTRAJECTORY(thAnchors,rAnchors,Delta,planner) uses a 
%   set of ordered anchor points (defined in polar coordinates) and an 
%   initial heading angle to plan a curvilinear trajectory through them.  
%   The input 'planner' structure contains parameters that specify the
%   temporal discretization and overall scaling of the trajectory.
%
%   See also: ORDERANCHORS, OPTIMIZETRAJECTORY, EXECUTETRAJECTORY 

% define generative model
velocity  = @(t,amplitude,T) amplitude.*.5*(1-cos(t.*2.*pi./T));
heading   = @(t,Delta,th,T) ((pi-2*Delta).*t./T+Delta+th-pi/2);
amplitude = @(r,Delta,T) (r./T).*(pi-2*Delta).*(pi+2*Delta).*(3*pi-2*Delta)./(4.*pi.^2.*cos(Delta));

xtraj = @(t,R,Delta,th,T) cumsum(velocity(t,amplitude(R,Delta,T),T).*cos(heading(t,Delta,th,T)));
ytraj = @(t,R,Delta,th,T) cumsum(velocity(t,amplitude(R,Delta,T),T).*sin(heading(t,Delta,th,T)));

% compute scaling
tt = planner.tAxis;
scale = ytraj(tt,1,Delta,pi/2,1);
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
        Delta = pi-(dth-dth0)-Delta0;
    end

    % generate trajectory linking anchors; choose multiple of initial  
    % heading offset that minimizes curvilinear distance
    Deltas = [Delta-2*pi,Delta,Delta+2*pi];
    dist   = nan(size(Deltas));
    [xtmp,ytmp] = deal(nan(size(Deltas,2),numel(tt)));
    for j=1:3
        xtmp(j,:) = x0+xtraj(tt,dr,Deltas(j),dth,T)./scale;
        ytmp(j,:) = y0+ytraj(tt,dr,Deltas(j),dth,T)./scale;
        dist(j)   = sum(dcart(xtmp(j,:),ytmp(j,:)));
    end
    [~,isel] = min(dist);

    % append trajectory
    trajectory.prevAnchor = [trajectory.prevAnchor,(i-1)*ones(1,numel(tt))];
    trajectory.xCoords    = [trajectory.xCoords,   xtmp(isel,:)];
    trajectory.yCoords    = [trajectory.yCoords,   ytmp(isel,:)];
    trajectory.velocity   = [trajectory.velocity,  velocity(tt,amplitude(dr,Deltas(isel),T),T)];
    trajectory.heading    = [trajectory.heading,   heading(tt,Deltas(isel),dth,T)];
    trajectory.amplitude  = [trajectory.amplitude, amplitude(dr,Deltas(isel),T)];
    trajectory.offset     = [trajectory.offset,    Deltas(isel)];

    % update initial condition
    x0     = xtmp(isel,end);
    y0     = ytmp(isel,end);
    t0     = t0+tt(end);
    dth0   = dth;
    Delta0 = Delta;
end

trajectory.thAnchors = thAnchors;
trajectory.rAnchors  = rAnchors;

end