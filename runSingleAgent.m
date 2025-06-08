function simResults = runSingleAgent(belief,sampler,planner,trial)
% RUNSINGLEAGENT Simulates an agent that learns to intercept a target.
%
%   simResults = RUNSINGLEAGENT(arena,belief,sampler,planner,trial) takes as 
%   input several structures that specify properties of the arena (in 'arena'), 
%   properties of the agent (in 'belief', 'sampler', and 'planner'), and 
%   the trial protocol (in 'trial'), and returns a structure that contains
%   the output of a simulated agent. 


%------------------- initialize priors, errormap, cache ------------------%
uniformTargetPrior = belief.uniformTargetPrior;
targetPosterior    = uniformTargetPrior;
targetErrormap     = belief.mask.*zeros(belief.np,belief.np);
cache              = nan(belief.np,belief.np,1);
cache(:,:,1)       = targetPosterior;
contextPosterior   = 1;
contextIDs         = 0;                      % key: 0 (current), N (newest), N-1, ... 1 (oldest)

%--------------------- initialize storage variables ----------------------%
[executedTargetLikelihoods,plannedTargetLikelihoods,...
    targetPosteriors,targetErrormaps                    ] = deal(nan(belief.np,belief.np,trial.nTrials));   
[traj_executed,traj_planned                             ] = deal(cell(1,trial.nTrials));
[contextPosteriors                                      ] = deal(nan(belief.cacheSize,trial.nTrials));
[outcome,reward,outcomeProb,rewardProb,...
    nAnchors_executed,nAnchors_planned,...
    distance_executed,distance_planned,...
    initAngle_planned,initAngle_executed,...
    targetPosteriorEntropy,contextPosteriorEntropy,...
    obstacleHit,resetFlag,cacheFlag,boundaryFlag,...
    sampledContext                                      ] = deal(nan(trial.nTrials,1));


%------------------------- run learning algorithm ------------------------%
for trialID=1:trial.nTrials

    % update prior beliefs
    contextPrior = contextPosterior;
    targetPrior  = targetPosterior;
    cache(:,:,1) = targetPrior;

    % select prior belief that will be used to sample anchor points
    [targetPriorToSample,sampledContext(trialID)] = sampleContext(contextPrior,contextIDs,cache,belief);

    % sample anchor points from prior
    anchors = sampleAnchors(targetPriorToSample,belief,sampler,planner);

    % plan optimal trajectory through the set of anchors
    plannedTrajectory = optimizeTrajectory(anchors,belief,planner);

    % evaluate whether planned trajectory is likely to intercept obstacles;
    % if so, augment anchor points to avoid obstacle
    plannedTrajectory = evaluateTrajectory(plannedTrajectory,targetErrormap,belief,sampler,planner);

    % execute trajectory; adjust based on arena boundaries and obstacles
    [executedTrajectory,obstacleHit(trialID)] = executeTrajectory(plannedTrajectory,trial,planner,trialID);
    boundaryFlag(trialID) = executedTrajectory.boundaryFlag;

    % use planned and executed trajectories to compute likelihood
    plannedTargetLikelihood  = getTargetLikelihood(plannedTrajectory, belief);
    executedTargetLikelihood = getTargetLikelihood(executedTrajectory,belief);

    % determine whether trajectory intercepted the target and get reward
    [outcome(trialID),reward(trialID)] = getOutcome(executedTrajectory,trial,trialID);

    % compute the probability of the different outcomes under the prior
    outcomeProb(trialID) = computeOutcomeProb(targetPrior,executedTargetLikelihood,outcome(trialID));
    rewardProb( trialID) = computeOutcomeProb(targetPrior,executedTargetLikelihood,       1        );

    % use the outcome surprise under the current prior, relative to a uniform prior,
    % to determine whether to reset posterior
    outcomeSurprise(trialID,1) = computeSurprise(outcomeProb(trialID))./...
        computeSurprise(computeOutcomeProb(uniformTargetPrior,executedTargetLikelihood,outcome(trialID)));

    % update errormap based on planned and executed trajectory
    targetErrormap = updateErrormap(plannedTargetLikelihood,executedTargetLikelihood,targetErrormap);

    % update belief based on executed trajectory and outcome
    [targetPosterior,contextPosterior,contextIDs,cache,resetFlag(trialID),cacheFlag(trialID)]...
        = updateBelief(targetPrior,executedTargetLikelihood,outcome(trialID),contextPrior,contextIDs,cache,outcomeSurprise,belief);

    % compute entropies of updated belief
    targetPosteriorEntropy( trialID) = computeEntropy(targetPosterior);
    contextPosteriorEntropy(trialID) = computeEntropy(contextPosterior);

    % apply memory decay to posterior beliefs and existing caches
    targetPosterior  = applyMemoryDecay(targetPosterior, uniformTargetPrior,belief);
    contextPosterior = applyMemoryDecay(contextPosterior,normalizeBelief(ones(size(contextPosterior))),belief);
    for j=2:size(cache,3)
        cache(:,:,j) = applyMemoryDecay(cache(:,:,j),uniformTargetPrior,belief);
    end

    %-------------------------- append results ---------------------------%
    executedTargetLikelihoods(:,:,trialID) = executedTargetLikelihood(:,:,outcome(trialID));
    plannedTargetLikelihoods( :,:,trialID) = plannedTargetLikelihood( :,:,outcome(trialID));
    targetPosteriors(         :,:,trialID) = targetPosterior;
    targetErrormaps(          :,:,trialID) = targetErrormap;
    contextPosteriors(1:numel(contextPosterior),trialID) = contextPosterior';
    
    % for planned trajectory, only store anchor points and initial heading 
    % (sufficient to recover full planned trajectory)
    traj_planned{ trialID}.anchors = plannedTrajectory.anchors;
    traj_planned{ trialID}.delta   = plannedTrajectory.delta;

    % store full executed trajectory
    traj_executed{trialID} = executedTrajectory;

    % store # anchors, initial angles, and path lengths in accessible field
    nAnchors_executed(trialID,1) = executedTrajectory.anchors.N;
    nAnchors_planned( trialID,1) = plannedTrajectory.anchors.N;

    distance_executed(trialID,1) = executedTrajectory.distance;
    distance_planned( trialID,1) = plannedTrajectory.distance;

    initAngle_executed(trialID,1) = executedTrajectory.anchors.thCoords(2);
    initAngle_planned( trialID,1) = plannedTrajectory.anchors.thCoords(2);

end

%--------------------------- reformat beliefs ----------------------------%
[contextPosteriors,estimatedContext,sampledContext,cache] ...
    = reformatContextBelief(contextPosteriors,sampledContext,cache,contextIDs,cacheFlag);

%----------------------------- store results -----------------------------%

simResults.trajectory.executed.path             = traj_executed;
simResults.trajectory.executed.nAnchors         = nAnchors_executed;
simResults.trajectory.executed.distance         = distance_executed;
simResults.trajectory.executed.initAngle        = initAngle_executed;

simResults.trajectory.planned.path              = traj_planned;
simResults.trajectory.planned.nAnchors          = nAnchors_planned;
simResults.trajectory.planned.distance          = distance_planned;
simResults.trajectory.planned.initAngle         = initAngle_planned;

simResults.trajectory.rewards                   = reward;
simResults.trajectory.obstacleHits              = obstacleHit;
simResults.trajectory.boundaryFlag              = boundaryFlag;

simResults.belief.target.initialPrior           = uniformTargetPrior;
simResults.belief.target.likelihoods.executed   = executedTargetLikelihoods;
simResults.belief.target.likelihoods.planned    = plannedTargetLikelihoods;
simResults.belief.target.posteriors             = targetPosteriors;
simResults.belief.target.errormaps              = targetErrormaps;
simResults.belief.target.outcomeSurprise        = outcomeSurprise;
simResults.belief.target.rewardProb             = rewardProb;
simResults.belief.target.posteriorEntropy       = targetPosteriorEntropy;
simResults.belief.target.resetFlag              = resetFlag;
simResults.belief.target.cacheFlag              = cacheFlag;

simResults.belief.context.cache                 = cache;
simResults.belief.context.posteriors            = contextPosteriors;
simResults.belief.context.sampled               = sampledContext;
simResults.belief.context.estimated             = estimatedContext;
simResults.belief.context.posteriorEntropy      = contextPosteriorEntropy;

end
