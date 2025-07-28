function plotMultiAgentResults_surpriseReset(multiAgentResults1,multiAgentResults2,agent,trial,plotParams)

figure;set(gcf,'color','w','units','normalized','Position',[.025,.025,.4,.7]);

nTrials  = trial.nTrials;
nAgents  = multiAgentResults1.nAgents;
blockIDs = unique(multiAgentResults1.trialProtocol.blockIDs);
tBlock   = numel(find(multiAgentResults1.trialProtocol.blockIDs==1));
nBlocks  = numel(blockIDs);
blockSwitches = [0,blockIDs]*tBlock+1;

% plot trial averages without surprise update
subplot(3,2,1);
yyaxis left
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults1.trajectory.rewardRate(inds,:),plotParams,'k','-','reward rate');
end
yyaxis right
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults1.belief.target.rewardProb(inds,:),plotParams,'r','--','predicted rate',0);
end
plotTswitch(blockIDs,tBlock,[0,1])
ylim([0,1])
xlim([0,nBlocks.*tBlock])

subplot(3,2,3);
yyaxis left
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults1.belief.target.posteriorEntropy(inds,:)./multiAgentResults1.belief.target.posteriorEntropyFlat,plotParams,'k','-','uncertainty (bits)',0);
end
plotTswitch(blockIDs,tBlock,[0,1])
yyaxis right
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults1.belief.target.outcomeSurprise(inds,:),plotParams,'r','--','surprise (bits)',0);
end
plot([0,nTrials],[agent.belief.surpriseThreshold,agent.belief.surpriseThreshold],'--k')
xlim([0,nBlocks.*tBlock])


% plot uncertainty and surprise
fCache = sum(multiAgentResults1.belief.target.resetFlag,2)./nAgents;
ii = find(fCache>0);
scatter(ii,2.*ones(1,numel(ii)),fCache(ii)*200,'k',...
    'filled','MarkerFaceAlpha',.3)


% plot trial averages with surprise update
subplot(3,2,2);
yyaxis left
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults2.trajectory.rewardRate(inds,:),plotParams,'k','-','reward rate');
end
yyaxis right
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults2.belief.target.rewardProb(inds,:),plotParams,'r','--','predicted reward');
end
plotTswitch(blockIDs,tBlock,[0,1])
ylim([0,1])
xlim([0,nBlocks.*tBlock])

subplot(3,2,4);hold on;
yyaxis left
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults2.belief.target.posteriorEntropy(inds,:)./multiAgentResults2.belief.target.posteriorEntropyFlat,plotParams,'k','-','uncertainty (bits)');
end
plotTswitch(blockIDs,tBlock,[0,1])
yyaxis right
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults2.belief.target.outcomeSurprise(inds,:),plotParams,'r','--','surprise (bits)');
end
plot([0,nTrials],[agent.belief.surpriseThreshold,agent.belief.surpriseThreshold],'--k')
xlim([0,nBlocks.*tBlock])



% plot arena and target
subplot(3,2,5);
fill(trial.arena.xBoundary,trial.arena.yBoundary,plotParams.cArena,'linestyle','none');hold on;
plot(trial.arena.xBoundary,trial.arena.yBoundary,'color',plotParams.cBoundary,'linewidth',plotParams.lw)
for i=1:nBlocks
    fill(trial.target.xBoundary(i,:),trial.target.yBoundary(i,:),plotParams.cTarget,'linestyle','none')
end
daspect([1,1,1])
axis off
end

function plotTswitch(blockIDs,tBlock,yRange)
tSwitch = 1;
for i=1:numel(blockIDs)
    plot([tSwitch,tSwitch],yRange,'--k');
    tSwitch = tSwitch + tBlock;
end
end
