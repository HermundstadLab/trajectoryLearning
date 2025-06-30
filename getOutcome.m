function [outcome,reward,latency] = getOutcome(trajectory,trial,indTrial)
% GETOUTCOME Compute the outcome of the executed trajectory. 
%   [outcome,reward] = GETOUTCOME(trajectory,trial,indTrial) determines 
%   whether the executed trajectory intercepted the current target on a 
%   given trial, and returns the outcome (1 or 2) and corresponding
%   reward (1 or 0). The outcome is used to indexe other structures (like 
%   the likelihood) that store outcome-dependent results.
%   KEY:  outcome = 1: trajectory intercepted target (reward = 1)
%         outcome = 2: trajectory did not intercept target (reward = 0)

blockID = trial.blockIDs(indTrial);
latency = find(trajectory.xCoords>trial.target.xCenters(blockID)-trial.target.width/2 ...
        & trajectory.xCoords<trial.target.xCenters(blockID)+trial.target.width/2 ...
        & trajectory.yCoords>trial.target.yCenters(blockID)-trial.target.height/2 ...
        & trajectory.yCoords<trial.target.yCenters(blockID)+trial.target.height/2,1,'first');

if numel(latency)>0
    outcome = 1;
else
    outcome = 2;
    latency = nan;
end

reward = -outcome+2;

end