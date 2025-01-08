function plotSingleTrial(res,arena,belief,trial,trialID,plotParams)
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
%   See also: PLOTBELIEF, PLOTANCHORS, PLOTTRAJECTORY

priors = cat(3,res.belief.prior,res.belief.posteriors(:,:,1:end-1));
posteriors  = res.belief.posteriors;
likelihoods = res.belief.likelihoods;
plannedTrajectory  = res.trajectory.planned{trialID};
executedTrajectory = res.trajectory.executed{trialID};

figure;set(gcf,'color','w','units','normalized','Position',[.05,.05,.9,.3])
% plot prior belief and sampled anchor points
subplot(1,4,1);plotBelief(squeeze(priors(:,:,trialID)),belief,plotParams,'prior')
hold on; plotAnchors(plannedTrajectory,arena,belief,plotParams,'polar')

% plot executed trajectory
subplot(1,4,2);plotTrajectory(executedTrajectory,arena,trial,trialID,plotParams,'cart')
hold on; plotAnchors(executedTrajectory,arena,belief,plotParams,'cart')
title(['reward = ',num2str(res.trajectory.rewards(trialID))])
set(gca,'fontsize',plotParams.fs)

% plot empricial likelihood derived from executed trajectory
subplot(1,4,3);plotBelief(squeeze(likelihoods(:,:,trialID)),belief,plotParams,'likelihood');clim([0,1])
hold on; plotTrajectory(executedTrajectory,arena,trial,trialID,plotParams,'polar')
plotAnchors(executedTrajectory,arena,belief,plotParams,'polar')

% plot updated posterior belief
subplot(1,4,4);plotBelief(squeeze(posteriors(:,:,trialID)),belief,plotParams,'posterior')

