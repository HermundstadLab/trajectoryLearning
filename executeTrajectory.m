function [executedTrajectory,obstacleHit] = executeTrajectory(trajectory,arena,trial,planner,trialID)
% EXECUTETRAJECTORY Execute planned trajectory, accounting for arena
% boundaries and obstacles.
%
%   executedTrajectory = EXECUTETRAJECTORY(trajectory,arena,trial,planner,trialID)
%   takes as input a planned trajectory, and executes a version of the
%   trajectory that is consistent with arena boundaries and obstacles.
%   General and trial-specific properties of arena and obstacle boundaries
%   are conveyed through the input structures 'arena' and 'trial',
%   respectively. The input structure 'planner' is needed to divert the
%   trajectory around obstacles, if applicable.
%
%   See also: PLANTRAJECTORY, BOUNDTRAJECTORY, DIVERTTRAJECTORY


% determine whether trajectory passed through obstacle
obstacleHit = intersectTrajectory(trajectory.xCoords,trajectory.yCoords,...
    trial.obstacle.xBoundsTrue(trial.blockIDs(trialID),:),...
    trial.obstacle.yBoundsTrue(trial.blockIDs(trialID),:));

% trajectory passed through obstacle, divert trajectory around it
if obstacleHit
    executedTrajectory = divertTrajectory(trajectory,trial,planner,trialID);
else
    executedTrajectory = trajectory;
end

% bound portions of trajectory that exceed edges of arena
executedTrajectory = boundTrajectory(executedTrajectory,arena.xBounds,arena.yBounds);

end




