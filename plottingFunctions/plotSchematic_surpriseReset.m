function plotSchematic_surpriseReset(singleAgentResults,environment,agent,trial,trialID,plotParams)

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;

arena   = environment.arena;

%---------------- extract belief for individual trials -------------------%

priors = cat(3,singleAgentResults.belief.target.initialPrior,singleAgentResults.belief.target.posteriors(:,:,1:end-1));
prior_beforeSwitch = squeeze(priors(:,:,trialID));
executedTrajectory_beforeSwitch = singleAgentResults.trajectory.executed.path{trialID};

%----------------------------- plot results ------------------------------%
figure;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.5,.8]);

% plot prior belief and executed trajectory
subplot(2,2,1);
plotBelief(prior_beforeSwitch,belief,plotParams,'prior');
hold on;plotAnchors(executedTrajectory_beforeSwitch,arena,belief,sampler,planner,plotParams,'polar');
plotTrajectory(executedTrajectory_beforeSwitch,arena,planner,trial,trialID,plotParams,'planned','polar');

subplot(2,2,2);
plotTrajectory(executedTrajectory_beforeSwitch,arena,planner,trial,trialID,plotParams,'executed','cart');

subplot(2,2,3);
plotBelief(prior_beforeSwitch,belief,plotParams,'prior');
hold on;plotAnchors(executedTrajectory_beforeSwitch,arena,belief,sampler,planner,plotParams,'polar');
plotTrajectory(executedTrajectory_beforeSwitch,arena,planner,trial,trialID+1,plotParams,'planned','polar');

subplot(2,2,4);
plotTrajectory(executedTrajectory_beforeSwitch,arena,planner,trial,trialID+1,plotParams,'executed','cart');

