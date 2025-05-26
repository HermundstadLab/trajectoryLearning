function [anchors,peaks] = plotAnchors(trajectory,arena,belief,sampler,planner,plotParams,coordFrame,candAnchors,prior)
% PLOTANCHORS Plot a single trajectory embedded in the arena.
%
%   PLOTANCHORS(trajectory,arena,trial,trialID,plotParams,coordFrame) plots 
%   the agent's anchor points in either cartesian or polar coordinates. This 
%   is specified by the string input 'coordFrame', which can take values 
%   'cart' or 'polar'. 
%
%   See also: PLOTBELIEF, PLOTTRAJECTORY, PLOTCONTROLPARAMS, PLOTSINGLETRIAL
 
if nargin<9
    candAnchors = false;
    prior   = [];
    peaks   = [];
    anchors = trajectory.anchors;
    if nargin<7
        coordFrame = 'cart';
    end
end

%----------------------------- plot results ------------------------------%
if strcmp(coordFrame,'cart')
    
    if candAnchors
        % plot candidate anchor points
        [anchors,peaks] = sampleAnchors(prior,belief,sampler,planner);
        [peaks.xCoords,peaks.yCoords] = pol2cart(peaks.thCoords,peaks.rCoords);
        plot(peaks.xCoords,peaks.yCoords,'o','MarkerSize',plotParams.ms,...
            'markeredgecolor','none','markerfacecolor','w');hold on;

        % plot selected anchor points
        [anchors.xCoords,anchors.yCoords] = pol2cart(peaks.thCoords,peaks.rCoords);
        plot(anchors.xCoords,anchors.yCoords,'x','color',plotParams.cAnchor,...
            'MarkerSize',plotParams.ms,'linewidth',plotParams.lw)
    else
        % plot optimized anchor points
        [xCoords,yCoords] = pol2cart(trajectory.anchors.thCoords,trajectory.anchors.rCoords);
        plot(xCoords,yCoords,'x','color',plotParams.cAnchor,...
            'MarkerSize',plotParams.ms,'linewidth',plotParams.lw)
    end

    xlim(arena.xBounds)
    ylim(arena.yBounds)
    daspect([1,1,1])
    axis off

elseif strcmp(coordFrame,'polar')    

    % plot candidate anchor points
    if candAnchors
        [anchors,peaks] = sampleAnchors(prior,belief,sampler,planner);
        plot(pi-peaks.thCoords,peaks.rCoords,'o','MarkerSize',plotParams.ms,...
            'markeredgecolor','none','markerfacecolor','w');hold on;

        % plot selected anchor points
        plot(pi-anchors.thCoords,anchors.rCoords,'x','color',plotParams.cAnchor,...
            'MarkerSize',plotParams.ms,'linewidth',plotParams.lw)
    else

        % to facilitate comparison with cartesian coordinates, redefine angles
        % such that positive angles are defined clockwise from the -x axis, 
        % rather than counterclockwise from the +x axis.
        thCoords = pi-trajectory.anchors.thCoords;
        rCoords  = trajectory.anchors.rCoords;

        % plot optimized anchor points
        plot(thCoords,rCoords,'x','color',plotParams.cAnchor,'MarkerSize',plotParams.ms,'linewidth',plotParams.lw)
    end

    xlim(belief.thBounds)
    ylim(belief.rBounds)
    axis off

else
    error('unrecognized coordinate frame')
end
