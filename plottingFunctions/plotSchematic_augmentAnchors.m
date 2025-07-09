function plotSchematic_augmentAnchors(singleAgentResults,environment,agent,trial,trialID,plotParams)

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;

arena   = environment.arena;


%---------------- extract belief for individual trials -------------------%

priors = cat(3,singleAgentResults.belief.target.initialPrior,singleAgentResults.belief.target.posteriors(:,:,1:end-1));
prior  = squeeze(priors(:,:,trialID));

priorErrormaps = cat(3,singleAgentResults.belief.target.initialErrormap,singleAgentResults.belief.target.errormaps(:,:,1:end-1));
errormap  = squeeze(priorErrormaps(:,:,trialID));

executedTrajectory  = singleAgentResults.trajectory.executed.path{trialID};
augmentedTrajectory = singleAgentResults.trajectory.augmented.path{trialID};
plannedTrajectory   = singleAgentResults.trajectory.planned.path{  trialID};

anchorIDs = singleAgentResults.trajectory.augmented.path{trialID}.anchors.augmented;
augmentedAnchorsIDs = [1,find(anchorIDs>0.5),numel(anchorIDs)];

trimmedTrajectory.anchors.thCoords = augmentedTrajectory.anchors.thCoords(augmentedAnchorsIDs);
trimmedTrajectory.anchors.rCoords  = augmentedTrajectory.anchors.rCoords( augmentedAnchorsIDs);


%----------------------------- plot results ------------------------------%
figure;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.4,.95]);

% plot prior belief with planned trajectory
subplot(3,2,1);
plotBelief(prior,belief,plotParams,'prior');
plotTrajectory(plannedTrajectory,arena,planner,trial,trialID,plotParams,'planned','polar');

% plot errormap with planned trajectory
ax2 = subplot(3,2,2);
plotBelief(errormap,belief,plotParams,'error map');
colormap(ax2,redblue)
crange = max(abs(errormap(:)));
if crange>0
    clim([-crange,crange]);
end
hold on;plotAnchors(plannedTrajectory,arena,belief,sampler,planner,plotParams,'polar');
plotTrajectory(plannedTrajectory,arena,planner,trial,trialID,plotParams,'planned','polar');

% plot prior belief with planned anchor points
subplot(3,2,3);
plotBelief(prior,belief,plotParams,'prior');
hold on;plotAnchors(plannedTrajectory,arena,belief,sampler,planner,plotParams,'polar');
hold on;plotAnchors(plannedTrajectory,arena,belief,sampler,planner,plotParams,'polar');

% plot errormap with augmented anchor points
ax2 = subplot(3,2,4);
plotBelief(errormap,belief,plotParams,'error map');
colormap(ax2,redblue)
crange = max(abs(errormap(:)));
if crange>0
    clim([-crange,crange]);
end
hold on;plotAnchors(trimmedTrajectory,arena,belief,sampler,planner,plotParams,'polar');

% plot augmented trajectory
subplot(3,2,5);
plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'executed','cart');
hold on;plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'cart');
