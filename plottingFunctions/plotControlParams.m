function plotControlParams(trajectory,planner,plotParams,trajType,paramType)
% PLOTCONTROLPARAMS Plot control parameters used to generate a trajectory.
%
%   PLOTCONTROLPARAMS(trajectory,planner,plotParams,trajType) 
%   plots the control parameters that generated an agent's planned or
%   executed trajectory (specified by the input variable 'trajType'); if 
%   planned, this function recomputes the planned trajectory from the 
%   initial heading and anchor points (using information in the 'planner' 
%   structure). 
%
%   See also: PLOTBELIEF, PLOTANCHORS, PLOTTRAJECTORY, PLOTSINGLETRIAL

% default option is to plot velocity
if nargin<5
    paramType = 'velocity';
end

% if input trajectory is a planned (rather than executed) trajectory,
% recompute full trajectory from initial heading and anchor points
if strcmp(trajType,'planned')
    trajectory = planTrajectory(trajectory.anchors,trajectory.phi,planner);
end

if strcmp(paramType,'velocity')
    plot(trajectory.timepts,trajectory.velocity,'color',plotParams.cTraj,'linewidth',plotParams.lw);hold on;
    ymin = 0;
    ymax = ceil(max(trajectory.velocity)/10)*10;
elseif strcmp(paramType,'heading')
    plot(trajectory.timepts,wrapToPi(trajectory.heading),'color',plotParams.cTraj,'linewidth',plotParams.lw);hold on;
    ymin = -pi;
    ymax = pi;
else
    error('unrecognized control parameter type')
end

for i=1:trajectory.anchors.N
    plot([trajectory.anchors.timepts(i),trajectory.anchors.timepts(i)],[ymin,ymax],'--','color',plotParams.cTraj)
end

ylabel(paramType)
xlabel('time')
xlim([0,max(trajectory.timepts)])
ylim([ymin,ymax]);
set(gca,'fontsize',plotParams.fs)
