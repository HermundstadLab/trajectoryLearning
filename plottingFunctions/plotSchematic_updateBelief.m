function plotSchematic_updateBelief(singleAgentResults,environment,agent,trial,trialID,plotParams)

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;

arena   = environment.arena;

%---------------- extract belief for individual trials -------------------%

priors = cat(3,singleAgentResults.belief.target.initialPrior,singleAgentResults.belief.target.posteriors(:,:,1:end-1));
prior  = squeeze(priors(:,:,trialID));

reward = singleAgentResults.trajectory.rewards(trialID);
if reward==1
    likelihood = cat(3,singleAgentResults.belief.target.likelihoods.executed(:,:,trialID),1-singleAgentResults.belief.target.likelihoods.executed(:,:,trialID));
else
    likelihood = cat(3,1-singleAgentResults.belief.target.likelihoods.executed(:,:,trialID),singleAgentResults.belief.target.likelihoods.executed(:,:,trialID));
end

outcome      = -reward+2;
altOutcome   = mod(outcome,2)+1;
posterior    = singleAgentResults.belief.target.posteriors(:,:,trialID);
altPosterior = updateBelief(prior,likelihood,altOutcome,[],1,nan(size(prior)),[],[],belief);
plotTitles   = {'reward','no reward'};

executedTrajectory = singleAgentResults.trajectory.executed.path{trialID};


%----------------------------- plot results ------------------------------%
figure;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.4,.95]);

% plot prior belief
subplot(3,2,1);
plotBelief(prior,belief,plotParams,'prior');

% plot likelihood for reward and no-reward cases, together with executed
% trajectory
subplot(3,2,3);
plotBelief(likelihood(:,:,outcome),belief,plotParams,['likelihood, ', plotTitles{outcome}]);
hold on;plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'polar');
plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'planned','polar');

subplot(3,2,4);
plotBelief(likelihood(:,:,altOutcome),belief,plotParams,['likelihood, ', plotTitles{altOutcome}]);
hold on;plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'polar');
plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'planned','polar');

% plot likelihood for reward and no-reward cases
subplot(3,2,5);
plotBelief(posterior,belief,plotParams,['posterior, ', plotTitles{outcome}]);

subplot(3,2,6);
plotBelief(altPosterior,belief,plotParams,['posterior, ', plotTitles{altOutcome}]);

