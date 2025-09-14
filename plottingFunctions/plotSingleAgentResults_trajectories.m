function plotSingleAgentResults_trajectories(res,environment,agent,trial,trialIDs,plotParams,plotType)

if nargin<7
    plotType = 'cart';
end

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;

arena   = environment.arena;


%----------------------------- plot results ------------------------------%
figure;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.95,.95]);
nTrials = numel(trialIDs);

if numel(trialIDs)<10
    nCols = numel(trialIDs);
    nRows = 1;
else
    nCols = 10;
    nRows = ceil(nTrials/nCols);
end

for i=1:nTrials
   
    trialID = trialIDs(i);
    executedTrajectory  = res.trajectory.executed.path{trialID};

    subplot(nRows,nCols,i)
    plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'executed',plotType);
    hold on; plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,plotType);
    set(gca,'fontsize',plotParams.fs);
end