function plotSingleAgentResults_trajectories(res,environment,agent,trial,trialIDs,plotParams)

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;

arena   = environment.arena;


%----------------------------- plot results ------------------------------%
figure;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.95,.4]);
nTrials = numel(trialIDs);

for i=1:nTrials
   
    trialID = trialIDs(i);
    executedTrajectory  = res.trajectory.executed.path{trialID};

    subplot(1,nTrials,i)
    plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'executed','cart');
    hold on; plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'cart');
    title(['reward = ',num2str(res.trajectory.rewards(trialID))]);
    set(gca,'fontsize',plotParams.fs);
end