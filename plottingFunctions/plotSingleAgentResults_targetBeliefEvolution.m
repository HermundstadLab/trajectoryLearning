function plotSingleAgentResults_targetBeliefEvolution(singleAgentResults,environment,agent,trial,plotParams)

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;

arena   = environment.arena;

%---------------- extract belief for individual trials -------------------%
priors = cat(3,singleAgentResults.belief.target.initialPrior,singleAgentResults.belief.target.posteriors(:,:,1:end-1));
errormaps = cat(3,zeros(size(singleAgentResults.belief.target.initialPrior)),singleAgentResults.belief.target.errormaps(:,:,1:end-1));


%----------------------------- plot results ------------------------------%
figure;
nTrials = 8;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.95,.7]);

for i=1:nTrials
   
    prior    = squeeze(priors(   :,:,i));
    errormap = squeeze(errormaps(:,:,i));
    augmentedTrajectory = singleAgentResults.trajectory.augmented.path{i};
    executedTrajectory  = singleAgentResults.trajectory.executed.path{ i};

    % plot belief maps
    ax1(i) = subplot(3,nTrials,i);
    plotBelief(prior,belief,plotParams);
    hold on;plotAnchors(augmentedTrajectory,arena,belief,sampler,planner,plotParams,'polar',true,prior,0.5);
    clim([0,.001])
    colormap(ax1(i),parula)

    % plot error maps
    ax2(i) = subplot(3,nTrials,nTrials+i);
    plotBelief(errormap,belief,plotParams);
    hold on;plotAnchors(augmentedTrajectory,arena,belief,sampler,planner,plotParams,'polar',true,errormap,1);
    clim([-1,1])
    colormap(ax2(i),redblue)

    % plot current executed trajectory
    subplot(3,nTrials,2*nTrials+i);
    plotTrajectory(executedTrajectory,arena,planner,trial,i,plotParams,'executed','cart');
    hold on; plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'cart');
    title(['reward = ',num2str(singleAgentResults.trajectory.rewards(i))]);
    set(gca,'fontsize',plotParams.fs);

end