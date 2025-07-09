function [learningSpeed, avgPerformance, r] = plotMultiAgentResults_avgPerformance(multiAgentResults,agent,trial,plotParams)
% PLOTRESULTS_AVGPERFORMANCE Plot average performance of a batch of agents.
%
%   PLOTRESULTS_AVGPERFORMANCE(multiAgentResults,agent,trial,plotParams) 
%   plots the average performance of a batch of agents. This includes the
%   actual and predicted reward rates (subplot 1), the uncertainty and
%   outcome surprise (subplot 2), and the number of anchors and path length
%   (subplot 3).
%
%   See also: PLOTBELIEF, PLOTANCHORS, PLOTCONTROLPARAMS, PLOTTRAJECTORY

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;


%---------------- plot trial-averaged performance over time --------------%
figure;set(gcf,'color','w','units','normalized','Position',[.025,.025,.3,.7]);

nTrials  = trial.nTrials;
nAgents  = multiAgentResults.nAgents;
blockIDs = unique(multiAgentResults.trialProtocol.blockIDs);
tBlock   = numel(find(multiAgentResults.trialProtocol.blockIDs==1));
nBlocks  = numel(blockIDs);
blockSwitches = [0,blockIDs]*tBlock+1;

cmap = plotParams.cBlocks;

% plot predicted and received reward rate
subplot(3,2,1);
yyaxis left
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    r = plotTrialAverages(inds,multiAgentResults.trajectory.rewardRate(inds,:),plotParams,cmap(i,:),'-','reward rate');
    for j = 1:nAgents
        inds2 = ceil(numel(inds)/2):numel(inds);
        rAvg(j) = mean(r(inds2,j));

        if max(r(:,j))<=0.75
            trialNo(j) = nTrials./nBlocks;
        else
            tcross = find(r(:,j)<0.75,1,'last');
            if numel(tcross)<1
                trialNo(j) = 1;
            else
                trialNo(j) = tcross;
            end
            
        end
    end
    learningSpeed(1,i)    = mean(trialNo);
    learningSpeed(2:3,i)  = quantile(trialNo,[.25,.75])';

    avgPerformance(1,i)   = mean(rAvg);
    avgPerformance(2:3,i) = quantile(rAvg,[.25,.75])';
end
hold on;plotTswitch(blockIDs,tBlock,[0,1])
yyaxis right
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults.belief.target.rewardProb(inds,:),plotParams,cmap(i,:),'--','pred reward');
end
% plot uncertainty and surprise
fCache = sum(multiAgentResults.belief.target.cacheFlag,2)./nAgents;
ii = find(fCache>0);

subplot(3,2,3);
yyaxis left
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults.belief.target.posteriorEntropy(inds,:)./...
        multiAgentResults.belief.target.posteriorEntropyFlat,plotParams,cmap(i,:),'-','uncertainty');
end
%plot([1,nTrials],[multiAgentResults.belief.target.posteriorEntropyFlat,multiAgentResults.belief.target.posteriorEntropyFlat],'--k');
yyaxis right
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults.belief.target.outcomeSurprise(inds,:),plotParams,cmap(i,:),'--','surprise');
end
plot([1,nTrials],[belief.surpriseThreshold,belief.surpriseThreshold],'--k')
plotTswitch(blockIDs,tBlock,[0,2])
scatter(ii,2.*ones(1,numel(ii)),fCache(ii)*nAgents,'k',...
    'filled','MarkerFaceAlpha',.3)

% plot number of anchors and distance along trajectory
subplot(3,2,5);
yyaxis left
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults.trajectory.nAnchors(inds,:)-2,plotParams,cmap(i,:),'-','no. anchors');
end
plot([1,nTrials],[1,1],'--k');
plotTswitch(blockIDs,tBlock,[0,10])
yyaxis right
for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;
    plotTrialAverages(inds,multiAgentResults.trajectory.distance(inds,:),plotParams,cmap(i,:),'--','path length');
end

% plot initial and final anchor points
subplot(3,2,2);hold on;
plot(trial.arena.xBoundary,trial.arena.yBoundary,'-k','linewidth',plotParams.lw);
scatter(multiAgentResults.trajectory.initialAnchors.xCoords,multiAgentResults.trajectory.initialAnchors.yCoords,...
    100,'k','filled','MarkerFaceAlpha',.3,'MarkerEdgeAlpha',.3)
daspect([1,1,1])
axis off

subplot(3,2,4);hold on;
plot(trial.arena.xBoundary,trial.arena.yBoundary,'-k','linewidth',plotParams.lw);
scatter(multiAgentResults.trajectory.finalAnchors.xCoords,multiAgentResults.trajectory.finalAnchors.yCoords,...
    100,'k','filled','MarkerFaceAlpha',.3,'MarkerEdgeAlpha',.3)
daspect([1,1,1])
axis off

% plot arena and target
subplot(3,2,6);
fill(trial.arena.xBoundary,trial.arena.yBoundary,plotParams.cArena,'linestyle','none');hold on;
plot(trial.arena.xBoundary,trial.arena.yBoundary,'color',plotParams.cBoundary,'linewidth',plotParams.lw)
for i=1:nBlocks
    fill(trial.target.xBoundary(i,:),trial.target.yBoundary(i,:),plotParams.cTarget,'linestyle','none')
end
if ~isempty(trial.obstacle.xBoundary)
    for i=1:nBlocks
        fill(trial.obstacle.xBoundary(i,:),trial.obstacle.yBoundary(i,:),plotParams.cObstacle,'linestyle','none')
    end
end
daspect([1,1,1])
axis off

%------------------------ plot speed of learning -------------------------%
figure;set(gcf,'color','w','units','normalized','Position',[.025,.025,.3,.4]);
for i=1:nBlocks
    plot([i,i],learningSpeed(2:3,i),'color',cmap(i,:),'linewidth',plotParams.lw); hold on;
    plot(i,learningSpeed(1,i),'o','MarkerFaceColor',cmap(i,:),'MarkerEdgeColor','none','markersize',plotParams.ms)
end
xlim([0,nBlocks+1])
ylim([0,100])
xlabel('target number')
ylabel('no. trials for r>0.75 ')
set(gca,'fontsize',plotParams.fs)
end

function plotTswitch(blockIDs,tBlock,yRange)
tSwitch = 1;
for i=1:numel(blockIDs)
    plot([tSwitch,tSwitch],yRange,'--k');
    tSwitch = tSwitch + tBlock;
end
end