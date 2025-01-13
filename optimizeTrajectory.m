function trajectory = optimizeTrajectory(thAnchors,rAnchors,belief,planner)
% OPTIMIZETRAJECTORY Plan and optimize a trajectory.
%
%   trajectory = OPTIMIZETRAJECTORY(thAnchors,rAnchors,belief,planner) uses 
%   a set of input anchor points (defined in polar coordinates) to plan an 
%   optimized curvilinear trajectory through them. This function first  
%   determines the optimal ordering of anchors that minimizes the pairwise
%   distance between them. Given this ordering, it then determines the 
%   initial heading angle and precise anchor locations (within some
%   tolerance of the original locations, as specified by the 'belief' and 
%   'planner' structures) that jointly minimize the total curvilinear 
%   distance along the trajectory.
%
%   See also: ORDERANCHORS, PLANTRAJECTORY


%---------------------- optimizing anchor order --------------------------%
% determine optimal ordering of anchors
[thAnchorsOrdered,rAnchorsOrdered] = orderAnchors(thAnchors,rAnchors,planner);


%-------------------- optimizing anchor locations ------------------------%
% determine specific placement of anchors, within some tolerance of their
% original locations
nAnchors = numel(thAnchorsOrdered);
drLB  = planner.tol_rShift*ones( 1,nAnchors);       % radial shift,  lower bound 
drUB  = planner.tol_rShift*ones( 1,nAnchors);       % radial shift,  upper bound 
dthLB = planner.tol_thShift*ones(1,nAnchors);       % angular shift, lower bound 
dthUB = dthLB;                                      % angular shift, upper bound 

% define bounds for optimization (defined by the maximum of the lower bound
% and the minimum boundaries of the arena, and similarly by the minimum of
% the upper bound and the maximum boundaries of the arena):
LB = max([[belief.thMin.*ones(1,nAnchors),belief.rMin.*ones(1,nAnchors),0];...
    [thAnchorsOrdered-dthLB,rAnchorsOrdered-drLB,0 ]],[],1);
UB = min([[belief.thMax.*ones(1,nAnchors),belief.rMax.*ones(1,nAnchors),pi];...
    [thAnchorsOrdered+dthUB,rAnchorsOrdered+drUB,pi]],[],1);

% define initial condition (add small uniform noise to original anchor locations)
Delta = pi*rand();                                        % initial guess for first heading angle
p0 = [thAnchorsOrdered + planner.tol_thShift.*rand(1,nAnchors)-planner.tol_thShift/2, ...
    rAnchorsOrdered    + planner.tol_rShift.*rand( 1,nAnchors)-planner.tol_rShift/2,...
    Delta];
e0  = errFnc([thAnchorsOrdered,rAnchorsOrdered,Delta]);

% optimize anchor placement
options = optimoptions('fmincon','display','off','Algorithm','sqp');
[pmin,emin] = fmincon(@errFnc,p0,[],[],[],[],LB,UB,[],options);

% accept optimization if final curvilinear distance is lower than initial
if emin<e0
    thAnchorsOpt = pmin(1:nAnchors);
    rAnchorsOpt  = pmin(nAnchors+1:end-1);
    DeltaOpt     = pmin(end);
    trajectory   = planTrajectory(thAnchorsOpt,rAnchorsOpt,DeltaOpt,planner);
else
    trajectory   = planTrajectory(thAnchorsOrdered,rAnchorsOrdered,Delta,planner);
end

    function err = errFnc(params)
        nA   = (numel(params)-1)/2;
        traj = planTrajectory(params(1:nA),params(nA+1:2*nA),params(end),planner);
        err  = traj.distance;
    end

end


