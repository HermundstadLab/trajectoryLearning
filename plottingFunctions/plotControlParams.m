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

    % set bounds
    ymin = 0;
    ymax = ceil(max(trajectory.velocity)/10)*10;

elseif strcmp(paramType,'heading')
    % wrap heading to [-pi,pi] for plotting
    heading = wrapToPi(trajectory.heading);

    % find discontinuities in heading
    jumps = find(abs(diff(heading))>pi); 

    if numel(jumps)>0
        % plot in segments around discontinuities
        % plot first segment:
        inds = 1:jumps(1);
        plot(trajectory.timepts(inds),heading(inds),'color',plotParams.cTraj,'linewidth',plotParams.lw);hold on;
        for i=2:numel(jumps)
            % plot intermediate segments:
            inds = jumps(i-1)+1:jumps(i);
            plot(trajectory.timepts(inds),heading(inds),'color',plotParams.cTraj,'linewidth',plotParams.lw)
        end
    
        % plot final segment
        inds = jumps(end)+1:numel(heading);
        plot(trajectory.timepts(inds),heading(inds),'color',plotParams.cTraj,'linewidth',plotParams.lw)
    else
        % plot entire trajectory
        plot(trajectory.timepts,heading,'color',plotParams.cTraj,'linewidth',plotParams.lw);hold on;
    end

    % set bounds
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
