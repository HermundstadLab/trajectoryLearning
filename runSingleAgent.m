function singleAgentResults = runSingleAgent(agent,trial)
% RUNSINGLEAGENT Simulates an agent that learns to intercept a target.
%
%   singleAgentResults = RUNSINGLEAGENT(agent,trial) takes as input two structures 
%   that specify properties of the agent (in 'agent', including 'belief', 
%   'sampler', and 'planner' modules) and trial protocol (in 'trial'), 
%   and returns a structure that contains the output of a simulated agent. 

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;


%--------------------- initialize storage variables ----------------------%
[executedTargetLikelihoods,plannedTargetLikelihoods,...
    augmentedTargetLikelihoods,targetPosteriors,...
    targetErrormaps                                     ] = deal(nan(belief.np,belief.np,trial.nTrials));   
[traj_executed,traj_augmented,traj_planned              ] = deal(cell(1,trial.nTrials));
[contextPosteriors                                      ] = deal(nan(belief.cacheSize,trial.nTrials));
[outcome,reward,latency,outcomeProb,rewardProb,...
    nAnchors_executed,nAnchors_augmented,nAnchors_planned,...
    distance_executed,distance_augmented,distance_planned,...
    initAngle_planned,initAngle_augmented,initAngle_executed,...
    targetPosteriorEntropy,contextPosteriorEntropy,...
    resetFlag,cacheFlag,boundaryFlag,...
    contextToRead,contextToWrite                        ] = deal(nan(trial.nTrials,1));
allCaches = nan(belief.cacheSize,belief.np,belief.np,trial.nTrials);

%------------------- initialize priors, errormap, cache ------------------%
uniformTargetPrior    = belief.uniformTargetPrior;
uniformTargetErrormap = belief.mask.*zeros(belief.np,belief.np);
targetErrormap        = uniformTargetErrormap;

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

    contextPosterior = applyMemoryDecay(contextPosterior,uniformContextPrior,true,belief);
    for j=1:belief.cacheSize
        cache(:,:,j) = applyMemoryDecay(cache(:,:,j),uniformTargetPrior,true,belief);
    end
    targetErrormap = applyMemoryDecay(targetErrormap,uniformTargetErrormap,false,belief);

    % update prior beliefs
    contextPrior = contextPosterior;
    targetPriorToWrite = cache(:,:,contextToWrite(trialID));

    % select prior belief that will be used to sample anchor points
    [targetPriorToRead,contextToRead(trialID)] = sampleContext(contextPrior,cache,belief);

    % sample anchor points from prior
    anchors = sampleAnchors(targetPriorToRead,belief,sampler,planner);

    % plan optimal trajectory through the set of anchors
    plannedTrajectory = optimizeTrajectory(anchors,belief,planner,trial,trialID);

    % evaluate whether planned trajectory is likely to intercept obstacles;
    % if so, augment anchor points to avoid obstacle
    augmentedTrajectory = evaluateTrajectory(plannedTrajectory,targetErrormap,belief,sampler,planner,trial,trialID);

    % execute trajectory; adjust based on arena boundaries and obstacles
    [executedTrajectory,obstacleHit(trialID)] = executeTrajectory(augmentedTrajectory,planner,trial,trialID);
    boundaryFlag(trialID) = executedTrajectory.boundaryFlag;

    % use planned and executed trajectories to compute likelihood
    plannedTargetLikelihood   = getTargetLikelihood(plannedTrajectory,  belief);
    augmentedTargetLikelihood = getTargetLikelihood(augmentedTrajectory,belief);
    executedTargetLikelihood  = getTargetLikelihood(executedTrajectory, belief);

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
    targetErrormap = updateErrormap(augmentedTargetLikelihood,executedTargetLikelihood,targetErrormap);

    % update belief based on executed trajectory and outcome
    [targetPosteriorToWrite,contextPosterior,contextToWrite(trialID+1),cache,resetFlag(trialID),cacheFlag(trialID)]...
        = updateBelief(targetPriorToWrite,executedTargetLikelihood,outcome(trialID),contextPrior,contextToWrite(trialID),cache,outcomeSurprise,obstacleHit,belief);

    % compute entropies of updated belief
    targetPosteriorEntropy( trialID) = computeEntropy(targetPosteriorToWrite);
    contextPosteriorEntropy(trialID) = computeEntropy(contextPosterior);

    %-------------------------- append results ---------------------------%
    executedTargetLikelihoods( :,:,trialID) = executedTargetLikelihood( :,:,outcome(trialID));
    plannedTargetLikelihoods(  :,:,trialID) = plannedTargetLikelihood(  :,:,outcome(trialID));
    augmentedTargetLikelihoods(:,:,trialID) = augmentedTargetLikelihood(:,:,outcome(trialID));
    targetPosteriors(          :,:,trialID) = targetPosteriorToWrite;
    targetErrormaps(           :,:,trialID) = targetErrormap;
    contextPosteriors(1:numel(contextPosterior),trialID) = contextPosterior';

    for i=1:belief.cacheSize
        allCaches(i,:,:,trialID) = cache(:,:,i);
    end
    
    % for planned and augmented trajectories, only store anchor points and 
    % initial heading (sufficient to recover full planned trajectory)
    traj_planned{  trialID}.anchors = plannedTrajectory.anchors;
    traj_planned{  trialID}.delta   = plannedTrajectory.delta;

    traj_augmented{trialID}.anchors = augmentedTrajectory.anchors;
    traj_augmented{trialID}.delta   = augmentedTrajectory.delta;

    % store full executed trajectory
    traj_executed{trialID} = executedTrajectory;

    % store # anchors, initial angles, and path lengths in accessible field
    nAnchors_executed( trialID,1)  = executedTrajectory.anchors.N;
    nAnchors_augmented(trialID,1)  = augmentedTrajectory.anchors.N;
    nAnchors_planned(  trialID,1)  = plannedTrajectory.anchors.N;

    distance_executed( trialID,1)  = executedTrajectory.distance;
    distance_augmented(trialID,1)  = augmentedTrajectory.distance;
    distance_planned(  trialID,1)  = plannedTrajectory.distance;

    initAngle_executed( trialID,1) = executedTrajectory.anchors.thCoords(2);
    initAngle_augmented(trialID,1) = augmentedTrajectory.anchors.thCoords(2);
    initAngle_planned(  trialID,1) = plannedTrajectory.anchors.thCoords(2);

end

%----------------------------- store results -----------------------------%

singleAgentResults.trajectory.executed.path             = traj_executed;
singleAgentResults.trajectory.executed.nAnchors         = nAnchors_executed;
singleAgentResults.trajectory.executed.distance         = distance_executed;
singleAgentResults.trajectory.executed.initAngle        = initAngle_executed;
singleAgentResults.trajectory.executed.latency          = latency.*planner.dt;

singleAgentResults.trajectory.augmented.path            = traj_augmented;
singleAgentResults.trajectory.augmented.nAnchors        = nAnchors_augmented;
singleAgentResults.trajectory.augmented.distance        = distance_augmented;
singleAgentResults.trajectory.augmented.initAngle       = initAngle_augmented;

singleAgentResults.trajectory.planned.path              = traj_planned;
singleAgentResults.trajectory.planned.nAnchors          = nAnchors_planned;
singleAgentResults.trajectory.planned.distance          = distance_planned;
singleAgentResults.trajectory.planned.initAngle         = initAngle_planned;

singleAgentResults.trajectory.rewards                   = reward;
singleAgentResults.trajectory.obstacleHits              = obstacleHit;
singleAgentResults.trajectory.boundaryFlag              = boundaryFlag;

singleAgentResults.belief.target.initialPrior           = uniformTargetPrior;
singleAgentResults.belief.target.likelihoods.executed   = executedTargetLikelihoods;
singleAgentResults.belief.target.likelihoods.augmented  = augmentedTargetLikelihoods;
singleAgentResults.belief.target.likelihoods.planned    = plannedTargetLikelihoods;
singleAgentResults.belief.target.posteriors             = targetPosteriors;
singleAgentResults.belief.target.initialErrormap        = uniformTargetErrormap;
singleAgentResults.belief.target.errormaps              = targetErrormaps;
singleAgentResults.belief.target.outcomeSurprise        = outcomeSurprise;
singleAgentResults.belief.target.rewardProb             = rewardProb;
singleAgentResults.belief.target.posteriorEntropy       = targetPosteriorEntropy;
singleAgentResults.belief.target.resetFlag              = resetFlag;
singleAgentResults.belief.target.cacheFlag              = cacheFlag;

singleAgentResults.belief.context.allCaches             = allCaches;
singleAgentResults.belief.context.posteriors            = contextPosteriors;
singleAgentResults.belief.context.toRead                = contextToRead;
singleAgentResults.belief.context.toWrite               = contextToWrite;
singleAgentResults.belief.context.posteriorEntropy      = contextPosteriorEntropy;

end
