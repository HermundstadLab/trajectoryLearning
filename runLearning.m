function simResults = runLearning(arena,belief,sampler,planner,trial)
% RUNLEARNING Simulates an agent that learns to intercept a target.
%
%   simResults = RUNLEARNING(arena,belief,sampler,planner,trial) takes as 
%   input several structures that specify properties of the arena (in 'arena'), 
%   properties of the agent (in 'belief', 'sampler', and 'planner'), and 
%   the trial protocol (in 'trial'), and returns a structure that contains
%   the output of a simulated agent. 


%--------------------- initialize prior and errormap ---------------------%
uniformPrior = normalizeBelief(belief.mask.*ones(belief.np,belief.np));
posterior    = uniformPrior;
errormap     = belief.mask.*zeros(belief.np,belief.np);


%--------------------- initialize storage variables ----------------------%
[likelihoods,posteriors,errormaps] = deal(nan(belief.np,belief.np,trial.nTrials));   
[traj_executed,traj_planned]       = deal(cell(1,trial.nTrials));
[outcome,reward,probOutcome,...
    probReward]                    = deal(nan(trial.nTrials,1));
    

%------------------------- run learning algorithm ------------------------%
for trialID=1:trial.nTrials

    % update prior belief 
    prior = posterior;

    % sample anchor points from prior
    [thAnchors,rAnchors] = sampleAnchors(prior,belief,sampler);

    % plan optimal trajectory through the set of anchors
    plannedTrajectory = optimizeTrajectory(thAnchors,rAnchors,belief,planner);

    % evaluate whether planned trajectory is likely to intercept obstacles;
    % if so, augment anchor points to avoid obstacle
    plannedTrajectory = evaluateTrajectory(plannedTrajectory,errormap,belief,sampler,planner);

    % execute trajectory; adjust based on arena boundaries and obstacles
    executedTrajectory = executeTrajectory(plannedTrajectory,arena,trial,planner,trialID);

    % use planned and executed trajectories to compute  likelihood
    plannedLikelihood  = getLikelihood(plannedTrajectory, belief);
    executedLikelihood = getLikelihood(executedTrajectory,belief);

    % determine whether trajectory intercepted the target and get reward
    [outcome(trialID),reward(trialID)] = getOutcome(executedTrajectory,trial,trialID);

    % compute the probability of the different outcomes under the prior
    probOutcome(trialID) = computeOutcomeProb(prior,executedLikelihood,outcome(trialID));
    probReward(trialID)  = computeOutcomeProb(prior,executedLikelihood,   1      );

    % use the outcome surprise to determine whether to cache posterior
    cacheSignal(trialID,1) = computeSurprise(probOutcome(trialID));

    % update errormap based on planned and executed trajectory
    errormap = updateErrormap(plannedLikelihood,executedLikelihood,errormap);

    % update belief based on executed trajectory and outcome
    posterior = updateBelief(prior,executedLikelihood,outcome(trialID),cacheSignal,belief);

    %------------------------ append results ---------------------------%
    likelihoods(:,:,trialID) = executedLikelihood(:,:,outcome(trialID));
    posteriors( :,:,trialID) = posterior;
    errormaps(  :,:,trialID) = errormap;
    
    % for planned trajectory, only store anchor points and initial heading 
    % (sufficient to recover full planned trajectory)
    plannedTrajectoryCompact.thAnchors = plannedTrajectory.thAnchors;
    plannedTrajectoryCompact.rAnchors  = plannedTrajectory.rAnchors;
    plannedTrajectoryCompact.phi       = plannedTrajectory.phi;

    traj_executed{trialID} = executedTrajectory;
    traj_planned{ trialID} = plannedTrajectoryCompact;

end

%--------------------------- store results -----------------------------%
simResults.trajectory.executed  = traj_executed;
simResults.trajectory.planned   = traj_planned;
simResults.trajectory.rewards   = reward;

simResults.belief.prior         = uniformPrior;
simResults.belief.likelihoods   = likelihoods;
simResults.belief.posteriors    = posteriors;
simResults.belief.cacheSignal   = cacheSignal;
simResults.belief.probReward    = probReward;

end
