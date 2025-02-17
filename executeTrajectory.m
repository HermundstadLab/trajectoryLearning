function [executedTrajectory,obstacleHit] = executeTrajectory(trajectory,trial,planner,trialID)
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

% determine whether trajectory would pass through obstacle
obstacleHit = intersectTrajectory(trajectory.xCoords,trajectory.yCoords,...
    trial.obstacle.xBounds(trial.blockIDs(trialID),:),...
    trial.obstacle.yBounds(trial.blockIDs(trialID),:),'inside');

% if trajectory would hit obstacle, divert trajectory around it
if obstacleHit
    executedTrajectory = divertTrajectory(trajectory,trial,planner,trialID);
else
    executedTrajectory = trajectory;
end

end




