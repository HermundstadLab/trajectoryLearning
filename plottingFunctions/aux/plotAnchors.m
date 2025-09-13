function [anchors,peaks] = plotAnchors(trajectory,arena,belief,sampler,planner,plotParams,coordFrame,candAnchors,prior,samplingFrac)
% PLOTANCHORS Plot a single trajectory embedded in the arena.
%
%   PLOTANCHORS(trajectory,arena,trial,trialID,plotParams,coordFrame) plots 
%   the agent's anchor points in either cartesian or polar coordinates. This 
%   is specified by the string input 'coordFrame', which can take values 
%   'cart' or 'polar'. 
%
%   See also: PLOTBELIEF, PLOTTRAJECTORY, PLOTCONTROLPARAMS, PLOTSINGLETRIAL
 
if nargin<10
    samplingFrac = 0.5;
    candAnchors = false;
    prior   = belief.uniformTargetPrior;
    peaks   = [];
    anchors = trajectory.anchors;
    if nargin<7
        coordFrame = 'cart';
    end
end

%----------------------------- plot results ------------------------------%
if strcmp(coordFrame,'cart')
    
    DKL = computeKLdiv(normalizeMap(prior),belief.uniformTargetPrior);

    if candAnchors && DKL/belief.baseEntropy > sampler.uniformPriorThreshold
        % plot candidate anchor points     
        [anchors,peaks] = sampleAnchors(prior,belief,sampler,planner,samplingFrac);
        [peaks.xCoords,peaks.yCoords] = pol2cart(peaks.thCoords,peaks.rCoords);
        plot(peaks.xCoords,peaks.yCoords,'o','MarkerSize',plotParams.ms,...
            'markeredgecolor','none','markerfacecolor','w');hold on;

        % plot sampled anchor points
        [xCoords,yCoords] = pol2cart(anchors.thCoords,anchors.rCoords);
        plot(xCoords,yCoords,'x','color',plotParams.cAnchor,...
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

elseif strcmp(coordFrame,'cartScatter')
   
    % plot optimized anchor points
    [xCoords,yCoords] = pol2cart(trajectory.anchors.thCoords,trajectory.anchors.rCoords);
    scatter(xCoords,yCoords,50,'k','filled')

    xlim(arena.xBounds)
    ylim(arena.yBounds)
    daspect([1,1,1])
    axis off
    
elseif strcmp(coordFrame,'polar')    

    DKL = computeKLdiv(normalizeMap(prior),belief.uniformTargetPrior);

    % plot candidate anchor points
    if candAnchors && DKL/belief.baseEntropy > sampler.uniformPriorThreshold
        [anchors,peaks] = sampleAnchors(prior,belief,sampler,planner,samplingFrac);
        plot(pi-peaks.thCoords,peaks.rCoords,'o','MarkerSize',plotParams.ms,...
            'markeredgecolor','none','markerfacecolor','w');hold on;

        % plot sampled anchor points
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
