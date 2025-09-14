function plotMultiAgentResults_contextBelief(multiAgentResults,previousContexts,sampledContexts,plotParams)
% PLOTMULTIAGENTRESULTS_CONTEXTBELIEF Plot average performance of a batch of agents.
%
%   PLOTMULTIAGENTRESULTS_CONTEXTBELIEF(multiAgentResults,previousContexts,sampledContexts,plotParams)
%   plots the evolution of a context belief average for a batch of agents. 
%   This includes the belief over time, averaged across all agents (subplot 1), 
%   and the times at which individual agents sampled previous beliefs after
%   the context has switched (subplot 2).
%
%   See also: PLOTSINGLEAGENTRESULTS_CONTEXTBELIEFEVOLUTION

nAgents = size(previousContexts,1);
cmap = parula(5);

figure;set(gcf,'color','w','units','normalized','Position',[.025,.025,.5,.7]);

subplot(2,1,1);hold on;
belief = squeeze(mean(multiAgentResults.belief.context.posteriors,3));
for i=1:5
    plot(1:500,belief(i,:),'linewidth',plotParams.lw,'color',cmap(i,:))
end

set(gca,'fontsize',plotParams.fs)
xlabel('trials');
ylabel('context belief');

subplot(2,1,2);hold on;
for i=nAgents:-1:1
    ii = find(previousContexts(i,:)-sampledContexts(i,:)>0);
    scatter(ii,i*ones(1,numel(ii)),50,sampledContexts(i,ii),'filled');
    colormap(cmap);clim([1,5])
end
xlim([0,500])
plot([101,101],[1,500],'--k')
plot([201,201],[1,500],'--k')
plot([301,301],[1,500],'--k')
plot([401,401],[1,500],'--k')
set(gca,'fontsize',plotParams.fs)
xlabel('trials');
ylabel('agents');