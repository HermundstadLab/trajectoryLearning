function simResults = runSingleAgent(belief,sampler,planner,trial)
% RUNSINGLEAGENT Simulates an agent that learns to intercept a target.
%
%   simResults = RUNSINGLEAGENT(arena,belief,sampler,planner,trial) takes as 
%   input several structures that specify properties of the arena (in 'arena'), 
%   properties of the agent (in 'belief', 'sampler', and 'planner'), and 
%   the trial protocol (in 'trial'), and returns a structure that contains
%   the output of a simulated agent. 

%--------------------- initialize storage variables ----------------------%
[executedTargetLikelihoods,plannedTargetLikelihoods,...
    targetPosteriors,targetErrormaps                    ] = deal(nan(belief.np,belief.np,trial.nTrials));   
[traj_executed,traj_planned                             ] = deal(cell(1,trial.nTrials));
[contextPosteriors                                      ] = deal(nan(belief.cacheSize,trial.nTrials));
[outcome,reward,latency,outcomeProb,rewardProb,...
    nAnchors_executed,nAnchors_planned,...
    distance_executed,distance_planned,...
    initAngle_planned,initAngle_executed,...
    targetPosteriorEntropy,contextPosteriorEntropy,...
    obstacleHit,resetFlag,cacheFlag,boundaryFlag,...
    contextToRead,contextToWrite                        ] = deal(nan(trial.nTrials,1));
allCaches = nan(belief.cacheSize,belief.np,belief.np,trial.nTrials);

%------------------- initialize priors, errormap, cache ------------------%
uniformTargetPrior = belief.uniformTargetPrior;
targetErrormap     = belief.mask.*zeros(belief.np,belief.np);

cache = [];
for i=1:belief.cacheSize
    cache = cat(3,cache,uniformTargetPrior);
end

contextToWrite(1) = 1;
contextPosterior  = zeros(1,belief.cacheSize);
contextPosterior(contextToWrite(1)) = 1;

%------------------------- run learning algorithm ------------------------%
for trialID=1:trial.nTrials

    % apply memory decay to posterior beliefs and existing caches
    uniformContextPrior = zeros(size(contextPosterior));
    uniformContextPrior(contextPosterior>0) = 1;
    uniformContextPrior = normalizeBelief(uniformContextPrior);

    contextPosterior = applyMemoryDecay(contextPosterior,uniformContextPrior,belief);
    for j=1:belief.cacheSize
        cache(:,:,j) = applyMemoryDecay(cache(:,:,j),uniformTargetPrior,belief);
    end

    % update prior beliefs
    contextPrior = contextPosterior;
    targetPriorToWrite = cache(:,:,contextToWrite(trialID));

    % select prior belief that will be used to sample anchor points
    [targetPriorToRead,contextToRead(trialID)] = sampleContext(contextPrior,cache,belief);

    % sample anchor points from prior
    anchors = sampleAnchors(targetPriorToRead,belief,sampler,planner);

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
    [outcome(trialID),reward(trialID),latency(trialID)] = getOutcome(executedTrajectory,trial,trialID);

    % compute the probability of the different outcomes under the prior
    outcomeProb(trialID) = computeOutcomeProb(targetPriorToWrite,executedTargetLikelihood,outcome(trialID));
    rewardProb( trialID) = computeOutcomeProb(targetPriorToWrite,executedTargetLikelihood,       1        );

    % use the outcome surprise under the current prior, relative to a uniform prior,
    % to determine whether to reset posterior
    outcomeSurprise(trialID,1) = computeSurprise(outcomeProb(trialID))./...
        computeSurprise(computeOutcomeProb(uniformTargetPrior,executedTargetLikelihood,outcome(trialID)));

    % update errormap based on planned and executed trajectory
    targetErrormap = updateErrormap(plannedTargetLikelihood,executedTargetLikelihood,targetErrormap);

    % update belief based on executed trajectory and outcome
    [targetPosteriorToWrite,contextPosterior,contextToWrite(trialID+1),cache,resetFlag(trialID),cacheFlag(trialID)]...
        = updateBelief(targetPriorToWrite,executedTargetLikelihood,outcome(trialID),contextPrior,contextToWrite(trialID),cache,outcomeSurprise,belief);

    % compute entropies of updated belief
    targetPosteriorEntropy( trialID) = computeEntropy(targetPosteriorToWrite);
    contextPosteriorEntropy(trialID) = computeEntropy(contextPosterior);

    %-------------------------- append results ---------------------------%
    executedTargetLikelihoods(:,:,trialID) = executedTargetLikelihood(:,:,outcome(trialID));
    plannedTargetLikelihoods( :,:,trialID) = plannedTargetLikelihood( :,:,outcome(trialID));
    targetPosteriors(         :,:,trialID) = targetPosteriorToWrite;
    targetErrormaps(          :,:,trialID) = targetErrormap;
    contextPosteriors(1:numel(contextPosterior),trialID) = contextPosterior';

    for i=1:belief.cacheSize
        allCaches(i,:,:,trialID) = cache(:,:,i);
    end
    
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

%----------------------------- store results -----------------------------%

simResults.trajectory.executed.path             = traj_executed;
simResults.trajectory.executed.nAnchors         = nAnchors_executed;
simResults.trajectory.executed.distance         = distance_executed;
simResults.trajectory.executed.initAngle        = initAngle_executed;
simResults.trajectory.executed.latency          = latency.*planner.dt;

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

simResults.belief.context.allCaches             = allCaches;
simResults.belief.context.posteriors            = contextPosteriors;
simResults.belief.context.toRead                = contextToRead;
simResults.belief.context.toWrite               = contextToWrite;
simResults.belief.context.posteriorEntropy      = contextPosteriorEntropy;

end
