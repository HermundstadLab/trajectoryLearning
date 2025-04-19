function simResults = runSingleAgent(belief,sampler,planner,trial)
% RUNSINGLEAGENT Simulates an agent that learns to intercept a target.
%
%   simResults = RUNSINGLEAGENT(arena,belief,sampler,planner,trial) takes as 
%   input several structures that specify properties of the arena (in 'arena'), 
%   properties of the agent (in 'belief', 'sampler', and 'planner'), and 
%   the trial protocol (in 'trial'), and returns a structure that contains
%   the output of a simulated agent. 


%------------------- initialize priors, errormap, cache ------------------%
uniformPrior     = normalizeBelief(belief.mask.*ones(belief.np,belief.np));
posterior        = uniformPrior;
errormap         = belief.mask.*zeros(belief.np,belief.np);
cache            = nan(belief.np,belief.np,1);
cache(:,:,1)     = posterior;
contextPosterior = 1;
contextIDs       = [0;0];   % first row: indexed relative to final cache    
                            %   key: 0 (current), N (newest), N-1, ... 1 (oldest)
                            % second row: indexed relative to current context
                            %   key: 0 (current), 1 (newest), ..., N (oldest)

%--------------------- initialize storage variables ----------------------%
[executedLikelihoods,plannedLikelihoods,...
    sampledPriors,posteriors,errormaps     ] = deal(nan(belief.np,belief.np,trial.nTrials));   
[traj_executed,traj_planned                ] = deal(cell(1,trial.nTrials));
[contextPosteriors                         ] = deal(nan(belief.cacheSize,trial.nTrials));
[contextEstimates                          ] = deal(nan(2,trial.nTrials));
[outcome,reward,probOutcome,...
    probReward,nAnchors_executed,...
    nAnchors_planned,distance_executed,...
    distance_planned,initAngle_planned,...
    initAngle_executed,entropy,...
    obstacleHit,resetFlag,cacheFlag,...
    boundaryFlag                           ] = deal(nan(trial.nTrials,1));


%------------------------- run learning algorithm ------------------------%
for trialID=1:trial.nTrials

    % update prior beliefs
    contextPrior = contextPosterior;
    prior        = posterior;
    cache(:,:,1) = prior;

    % select prior belief that will be used to sample anchor points
    [priorToSample, contextEstimates(:,trialID)] = sampleContext(contextPrior,contextIDs,cache,belief);

    % sample anchor points from prior
    anchors = sampleAnchors(priorToSample,belief,sampler,planner);

    % plan optimal trajectory through the set of anchors
    plannedTrajectory = optimizeTrajectory(anchors,belief,planner);

    % evaluate whether planned trajectory is likely to intercept obstacles;
    % if so, augment anchor points to avoid obstacle
    plannedTrajectory = evaluateTrajectory(plannedTrajectory,errormap,belief,sampler,planner);

    % execute trajectory; adjust based on arena boundaries and obstacles
    [executedTrajectory,obstacleHit(trialID)] = executeTrajectory(plannedTrajectory,trial,planner,trialID);
    boundaryFlag(trialID) = executedTrajectory.boundaryFlag;

    % use planned and executed trajectories to compute likelihood
    plannedLikelihood  = getLikelihood(plannedTrajectory, belief);
    executedLikelihood = getLikelihood(executedTrajectory,belief);

    % determine whether trajectory intercepted the target and get reward
    [outcome(trialID),reward(trialID)] = getOutcome(executedTrajectory,trial,trialID);

    % compute the probability of the different outcomes under the prior
    probOutcome(trialID) = computeOutcomeProb(prior,executedLikelihood,outcome(trialID));
    probReward(trialID)  = computeOutcomeProb(prior,executedLikelihood,       1        );

    % use the outcome surprise under the current prior, relative to a uniform prior,
    % to determine whether to reset posterior
    surprise(trialID,1) = computeSurprise(probOutcome(trialID))./...
        computeSurprise(computeOutcomeProb(uniformPrior,executedLikelihood,outcome(trialID)));

    % update errormap based on planned and executed trajectory
    errormap = updateErrormap(plannedLikelihood,executedLikelihood,errormap);

    % update belief based on executed trajectory and outcome
    [contextIDs,contextPosterior,posterior,cache,resetFlag(trialID),cacheFlag(trialID)] = updateBelief(contextIDs,contextPrior,prior,executedLikelihood,outcome(trialID),surprise,cache,belief);

    % compute entropy of updated belief
    entropy(trialID) = computeEntropy(posterior);

    %------------------------ append results ---------------------------%
    executedLikelihoods(:,:,trialID) = executedLikelihood(:,:,outcome(trialID));
    plannedLikelihoods( :,:,trialID) = plannedLikelihood( :,:,outcome(trialID));
    sampledPriors(      :,:,trialID) = priorToSample;
    posteriors(         :,:,trialID) = posterior;
    errormaps(          :,:,trialID) = errormap;
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

%--------------------------- store results -----------------------------%


simResults.trajectory.executed.path      = traj_executed;
simResults.trajectory.executed.nAnchors  = nAnchors_executed;
simResults.trajectory.executed.distance  = distance_executed;
simResults.trajectory.executed.initAngle = initAngle_executed;

simResults.trajectory.planned.path       = traj_planned;
simResults.trajectory.planned.nAnchors   = nAnchors_planned;
simResults.trajectory.planned.distance   = distance_planned;
simResults.trajectory.planned.initAngle  = initAngle_planned;

simResults.trajectory.rewards            = reward;
simResults.trajectory.obstacleHits       = obstacleHit;
simResults.trajectory.boundaryFlag       = boundaryFlag;

simResults.belief.prior                  = uniformPrior;
simResults.belief.sampledPriors          = sampledPriors;
simResults.belief.likelihoods.executed   = executedLikelihoods;
simResults.belief.likelihoods.planned    = plannedLikelihoods;
simResults.belief.posteriors             = posteriors;
simResults.belief.contextPosteriors      = contextPosteriors;
simResults.belief.contextEstimates       = contextEstimates;
simResults.belief.contextIDs             = contextIDs;
simResults.belief.errormaps              = errormaps;
simResults.belief.cacheSignal            = surprise;
simResults.belief.probReward             = probReward;
simResults.belief.entropy                = entropy;
simResults.belief.cache                  = cache;
simResults.belief.resetFlag              = resetFlag;
simResults.belief.cacheFlag              = cacheFlag;

end
