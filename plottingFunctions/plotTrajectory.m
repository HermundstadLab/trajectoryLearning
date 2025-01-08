function plotTrajectory(trajectory,arena,trial,trialID,plotParams,coordFrame)
% PLOTTRAJECTORY Plot a single trajectory embedded in the arena.
%
%   PLOTTRAJECTORY(trajectory,arena,trial,trialID,plotParams,coordFrame) 
%   plots the agent's trajectory in either cartesian or polar coordinates. 
%   This is specified by the string input 'coordFrame', which can take 
%   values 'cart' or 'polar'. If specified  in cartesian coordinates, this 
%   function also plots the target and, if applicable, the obstable that 
%   the agent encountered on a given trial. The input structures 'arena' 
%   and 'trial' contain information for plotting the arena boundaries and 
%   the locations of the target and obstacle.
%
%   See also: PLOTBELIEF, PLOTANCHORS, PLOTSINGLETRIAL
 
if nargin<6
    coordFrame = 'cart';
end


blockID = trial.blockIDs(trialID);
if strcmp(coordFrame,'cart')
    
    % plot trajectory, target, and obstacle (if applicable)
    fill(arena.xBoundary,arena.yBoundary,plotParams.cArena,'linestyle','none');hold on;
    plot(arena.xBoundary,arena.yBoundary,'color',plotParams.cBoundary,'linewidth',plotParams.lw)
    fill(trial.target.xBoundary(blockID,:),trial.target.yBoundary(blockID,:),plotParams.cTarget,'linestyle','none')
    if ~isempty(trial.obstacle.xBoundary)
        fill(trial.obstacle.xBoundary(blockID,:),trial.obstacle.yBoundary(blockID,:),plotParams.cObstacle)
    end
    plot(trajectory.xCoords,trajectory.yCoords,'color',plotParams.cTraj,'linewidth',plotParams.lw)
    
    
    daspect([1,1,1])
    axis off

elseif strcmp(coordFrame,'polar')

    % to facilitate comparison with cartesian coordinates, redefine angles
    % such that positive angles are defined clockwise from the -x axis, 
    % rather than counterclockwise from the +x axis.
    [thCoords,rCoords] = cart2pol(trajectory.xCoords,trajectory.yCoords);
    thCoords = pi-thCoords;

    % plot trajectory    
    plot(thCoords,rCoords,'color',plotParams.cTraj,'linewidth',plotParams.lw)


else
    error('unrecognized coordinate frame')
end