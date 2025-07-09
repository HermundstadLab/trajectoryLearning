function plotSchematic_sampleAnchors(res,arena,belief,sampler,planner,trialID,plotParams)

%---------------- extract belief for individual trials -------------------%
priors = cat(3,res.belief.prior,res.belief.posteriors(:,:,1:end-1));
prior  = squeeze(priors(:,:,trialID));

%----------------------------- plot results ------------------------------%
figure;hold on;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.25,.7])

% plot prior belief
subplot(4,1,[1,2]);
plotBelief(prior,belief,plotParams,'prior');hold on;
[~,peaks] = plotAnchors([],arena,belief,sampler,planner,plotParams,'polar',true,prior);

% plot probability associate with each anchor point
subplot(4,1,3);hold on;
b = bar(1:numel(peaks.values),peaks.values);
b.FaceColor = 'flat';
ninterp = 100;
cmap  = parula(ninterp);
cinds = linspace(0,max(peaks.values),ninterp);
for i=1:numel(peaks.values)
    [~,ind] = min(abs(cinds-peaks.values(i)));
    b.CData(i,:) = cmap(ind,:);
end
xlm = xlim;
xticklabels({})
ylabel('prob.')
set(gca,'fontsize',plotParams.fs)

subplot(4,1,4);hold on;
plot(1:numel(peaks.values),cumsum(peaks.values),'-k');
scatter(1:numel(peaks.values),cumsum(peaks.values),100,peaks.values,'filled');clim([0,max(peaks.values)])
plot(xlm,[.5,.5],'--k')
xlim(xlm);
ylim([0,1])
xlabel('anchor number')
ylabel('cum. prob.')
set(gca,'fontsize',plotParams.fs)