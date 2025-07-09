function plotSingleAgentResults_contextBeliefEvolution(singleAgentResults,trial,plotParams)

cmap = plotParams.cBlocks;

blockIDs = unique(trial.blockIDs);
tBlock   = numel(find(trial.blockIDs==1));
nBlocks  = numel(blockIDs);
blockSwitches = [0,blockIDs]*tBlock+1;

figure;set(gcf,'color','w','units','normalized','Position',[.025,.025,.7,.3]);

for i=1:nBlocks
    inds = blockSwitches(i):blockSwitches(i+1)-1;

    subplot(3,nBlocks,1:nBlocks);hold on;
    plot(inds,singleAgentResults.trajectory.executed.initAngle(inds),'o','MarkerSize',plotParams.ms,...
        'MarkerFaceColor',cmap(i,:),'MarkerEdgeColor','none');

    subplot(3,nBlocks,nBlocks+1:2*nBlocks);hold on;
    plot(1:trial.nTrials,singleAgentResults.belief.context.posteriors(i,:),'Linewidth',plotParams.lw,...
        'Color',cmap(i,:));
    plot([inds(1)+4,inds(1)+4],[0,1],'--k')
    
    subplot(3,nBlocks,2*nBlocks+i);hold on;  
    for j=1:nBlocks
        h(j) = bar(j,singleAgentResults.belief.context.posteriors(j,inds(1)+4));
        set(h(j),'FaceColor', cmap(j,:));
    end
    ylim([0,1])
    xlim([.25,nBlocks+.75])
    xticks(1:nBlocks)
    xlabel('context')
    ylabel('probability')
    set(gca,'fontsize',plotParams.fs)
    title(['trial = ',num2str(inds(1)+4)])

end
subplot(3,nBlocks,1:nBlocks);
ylim([0,pi])
yticks([0,pi/2,pi])
yticklabels({'0','\pi/2','\pi'})
xlabel('trial')
ylabel('init angle (rad)')
set(gca,'fontsize',plotParams.fs)

subplot(3,nBlocks,nBlocks+1:2*nBlocks);
ylim([0,1])
yticks([0,0.5,1])
xlabel('trial')
ylabel('context posterior')
set(gca,'fontsize',plotParams.fs)