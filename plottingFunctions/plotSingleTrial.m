function plotSingleTrial(res,arena,belief,planner,trial,trialID,plotParams)
% PLOTSINGLETRIAL Plot the output of a single learning trial.
%
%   PLOTSINGLETRIAL(res,arena,belief,trial,trialID,plotParams) plots  
%   the evolution of a single trial, as specified by the input 'trialID'.
%   This includes plotting the prior belief and sampling of a set of
%   sanchor points (subplot 1), to the execution of a trajectory in the
%   arena (subplot 2), to the contruction of the likelihood (subplot 3),
%   to the update of the posterior (subplot 4). The input structures 
%   'arena ' and 'belief' contain information for plotting the bounds  
%   of the arena and belief.
%
%   See also: PLOTBELIEF, PLOTANCHORS, PLOTCONTROLPARAMS, PLOTTRAJECTORY

priors = cat(3,res.belief.prior,res.belief.posteriors(:,:,1:end-1));
posteriors  = res.belief.posteriors;
likelihoods = res.belief.likelihoods;

plannedTrajectory  = res.trajectory.planned{trialID};
executedTrajectory = res.trajectory.executed{trialID};

futurePlannedTrajectory  = res.trajectory.planned{trialID+1};
futureExecutedTrajectory = res.trajectory.executed{trialID+1};

figure;set(gcf,'color','w','units','normalized','Position',[.025,.025,.95,.5])

% plot current executed trajectory
subplot(2,5,2);plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'executed','cart')
hold on; plotAnchors(executedTrajectory,arena,belief,plotParams,'cart')
title(['reward = ',num2str(res.trajectory.rewards(trialID))])
set(gca,'fontsize',plotParams.fs)

% plot future executed trajectory
subplot(2,5,4);plotTrajectory(futureExecutedTrajectory,arena,planner,trial,trialID,plotParams,'executed','cart')
hold on; plotAnchors(futureExecutedTrajectory,arena,belief,plotParams,'cart')
title(['reward = ',num2str(res.trajectory.rewards(trialID+1))])
set(gca,'fontsize',plotParams.fs)

% plot prior belief and planned trajectory
subplot(2,5,6);plotBelief(squeeze(priors(:,:,trialID)),belief,plotParams,'prior')
hold on; plotAnchors(plannedTrajectory,arena,belief,plotParams,'polar')
hold on; plotTrajectory(plannedTrajectory,arena,planner,trial,trialID,plotParams,'planned','polar')

% plot empricial likelihood derived from executed trajectory
subplot(2,5,7);plotBelief(squeeze(likelihoods(:,:,trialID)),belief,plotParams,'likelihood');clim([0,1])
hold on; plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'executed','polar')

% plot updated posterior belief with sampled anchors
subplot(2,5,8);plotBelief(squeeze(posteriors(:,:,trialID)),belief,plotParams,'posterior')
hold on; plotAnchors(futurePlannedTrajectory,arena,belief,plotParams,'polar')
hold on; plotTrajectory(futurePlannedTrajectory,arena,planner,trial,trialID,plotParams,'planned','polar')

% plot velocity & heading over time
subplot(2,5,9); plotControlParams(futurePlannedTrajectory,planner,plotParams,'planned','velocity')
subplot(2,5,10);plotControlParams(futurePlannedTrajectory,planner,plotParams,'planned','heading')



