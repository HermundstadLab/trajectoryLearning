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


%---------------------- optimizing anchor order --------------------------%
% determine optimal ordering of anchors
anchorsOrdered = orderAnchors(anchors,planner);


%-------------------- optimizing anchor locations ------------------------%
% determine specific placement of anchors, within some tolerance of their
% original locations
nAnchors = anchorsOrdered.N;
drLB  = planner.tol_rShift*ones( 1,nAnchors);       % radial shift,  lower bound 
drUB  = planner.tol_rShift*ones( 1,nAnchors);       % radial shift,  upper bound 
dthLB = planner.tol_thShift*ones(1,nAnchors);       % angular shift, lower bound 
dthUB = dthLB;                                      % angular shift, upper bound 

% define bounds for optimization (defined by the maximum of the lower bound
% and the minimum boundaries of the arena, and similarly by the minimum of
% the upper bound and the maximum boundaries of the arena):
LB = max([[belief.thMin.*ones(1,nAnchors),...
    [belief.rMin,belief.rMinAnchors.*ones(1,nAnchors-2),belief.rMin],0];...
    [anchorsOrdered.thCoords-dthLB,anchorsOrdered.rCoords-drLB,0 ]],[],1);
UB = min([[belief.thMax.*ones(1,nAnchors),...
    belief.rMax.*ones(1,nAnchors),pi];...
    [anchorsOrdered.thCoords+dthUB,anchorsOrdered.rCoords+drUB,pi]],[],1);

% define initial condition (add small uniform noise to original anchor locations)
phi = pi*rand();                                     % initial guess for first heading angle
p0 = [anchorsOrdered.thCoords + planner.tol_thShift.*rand(1,nAnchors)-planner.tol_thShift/2, ...
    anchorsOrdered.rCoords    + planner.tol_rShift.*rand( 1,nAnchors)-planner.tol_rShift/2,...
    phi];
e0  = errFnc([anchorsOrdered.thCoords,anchorsOrdered.rCoords,phi]);

% optimize anchor placement
options = optimoptions('fmincon','display','off','Algorithm','sqp');
[pmin,emin] = fmincon(@errFnc,p0,[],[],[],[],LB,UB,[],options);

% accept optimization if final curvilinear distance is lower than initial
if emin<e0
    anchorsOpt.thCoords = pmin(1:nAnchors);
    anchorsOpt.rCoords  = pmin(nAnchors+1:end-1);
    anchorsOpt.N = nAnchors;
    phiOpt       = pmin(end);
    trajectory   = planTrajectory(anchorsOpt,phiOpt,planner);
else
    trajectory   = planTrajectory(anchorsOrdered,phi,planner);
end

    function err = errFnc(params)
        anch.N   = (numel(params)-1)/2;
        anch.thCoords = params(1:anch.N); 
        anch.rCoords  = params(anch.N+1:2*anch.N); 
        traj = planTrajectory(anch,params(end),planner);
        err  = traj.distance;
    end

end


