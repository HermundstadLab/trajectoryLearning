function plotAnchors(trajectory,arena,belief,plotParams,coordFrame)
% PLOTANCHORS Plot a single trajectory embedded in the arena.
%
%   PLOTANCHORS(trajectory,arena,trial,trialID,plotParams,coordFrame) plots 
%   the agent's anchor points in either cartesian or polar coordinates. This 
%   is specified by the string input 'coordFrame', which can take values 
%   'cart' or 'polar'. 
%
%   See also: PLOTBELIEF, ,PLOTTRAJECTORY, PLOTSINGLETRIAL
 
if nargin<4
    coordFrame = 'cart';
end

if strcmp(coordFrame,'cart')
    
    [xAnchors,yAnchors] = pol2cart(trajectory.thAnchors,trajectory.rAnchors);
    plot(xAnchors,yAnchors,'x','color',plotParams.cAnchor,'MarkerSize',plotParams.ms,'linewidth',plotParams.lw)
    
    xlim(arena.xBounds)
    ylim(arena.yBounds)
    daspect([1,1,1])
    axis off

elseif strcmp(coordFrame,'polar')

    % to facilitate comparison with cartesian coordinates, redefine angles
    % such that positive angles are defined clockwise from the -x axis, 
    % rather than counterclockwise from the +x axis.
    thAnchors = pi-trajectory.thAnchors;
    rAnchors  = trajectory.rAnchors;

    % plot anchor points
    plot(thAnchors,rAnchors,'x','color',plotParams.cAnchor,'MarkerSize',plotParams.ms,'linewidth',plotParams.lw)

    xlim(belief.thBounds)
    ylim(belief.rBounds)
    axis off

else
    error('unrecognized coordinate frame')
end