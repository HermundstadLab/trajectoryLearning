function plotSingleAgentResults_singleTrialEvolution(singleAgentResults,environment,agent,trial,trialID,plotParams)
% PLOTSCHEMATIC_SINGLETRIAL Plot the output of a single learning trial.
%
%   PLOTSCHEMATIC_SINGLETRIAL(singleAgentResults,arena,belief,trial,trialID,plotParams) 
%   plots the evolution of a single trial, as specified by the input 
%   'trialID'. This includes plotting the prior belief and sampling of a 
%   set of anchor points (subplot 1), to the execution of a trajectory in
%   the arena (subplot 2), to the contruction of the likelihood (subplot 3),
%   to the update of the posterior (subplot 4). The input structures 
%   'arena ' and 'belief' contain information for plotting the bounds  
%   of the arena and belief.
%
%   See also: PLOTBELIEF, PLOTANCHORS, PLOTCONTROLPARAMS, PLOTTRAJECTORY

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;

arena   = environment.arena;

%---------------- extract belief for individual trials -------------------%

priors     = cat(3,singleAgentResults.belief.target.initialPrior,singleAgentResults.belief.target.posteriors(:,:,1:end-1));

prior      = squeeze(priors(:,:,trialID));
posterior  = squeeze(singleAgentResults.belief.target.posteriors(:,:,trialID));
likelihood = squeeze(singleAgentResults.belief.target.likelihoods.executed(:,:,trialID));

executedTrajectory       = singleAgentResults.trajectory.executed.path{ trialID  };
futurePlannedTrajectory  = singleAgentResults.trajectory.augmented.path{trialID+1};
futureExecutedTrajectory = singleAgentResults.trajectory.executed.path{ trialID+1};

figure;set(gcf,'color',plotParams.cFig,'units','normalized','Position',[.025,.025,.95,.5]);


%----------------------------- plot results ------------------------------%

% plot current executed trajectory
subplot(2,7,2);plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'executed','cart');
hold on; plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'cart');
title(['reward = ',num2str(singleAgentResults.trajectory.rewards(trialID))]);
set(gca,'fontsize',plotParams.fs);


% plot future executed trajectory 
subplot(2,7,4);plotTrajectory(futureExecutedTrajectory,arena,planner,trial,trialID,plotParams,'executed','cart');
hold on; plotAnchors(futureExecutedTrajectory,arena,belief,sampler,planner,plotParams,'cart');
title(['reward = ',num2str(singleAgentResults.trajectory.rewards(trialID+1))]);
set(gca,'fontsize',plotParams.fs);


% plot prior belief and planned trajectory 
subplot(2,7,8);plotBelief(prior,belief,plotParams,'prior');


%plot empricial likelihood derived from executed trajectory 
subplot(2,7,9);plotBelief(likelihood,belief,plotParams,'likelihood');clim([0,1]);
hold on; plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'polar');
hold on; plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'executed','polar');


% plot updated posterior belief 
subplot(2,7,10);plotBelief(posterior,belief,plotParams,'posterior');


% plot anchor sampling 
subplot(2,7,11);
plotBelief(posterior,belief,plotParams,'sampled anchors');
hold on;plotAnchors(futurePlannedTrajectory,arena,belief,sampler,planner,plotParams,'polar',true,posterior);


% plot planned trajectory 
subplot(2,7,12);
plot(belief.thBoundary,belief.rBoundary,'color',plotParams.cBoundary,'linewidth',plotParams.lw);
hold on; plotAnchors(futurePlannedTrajectory,arena,belief,sampler,planner,plotParams,'polar');
plotTrajectory(futurePlannedTrajectory,arena,planner,trial,trialID,plotParams,'planned','polar');
set(gca,'fontsize',16);
title('planned trajectory');


% plot velocity & heading over time
subplot(2,7,13); plotControlParams(futurePlannedTrajectory,planner,plotParams,'planned','velocity');title('control params')
subplot(2,7,14); plotControlParams(futurePlannedTrajectory,planner,plotParams,'planned','heading');



