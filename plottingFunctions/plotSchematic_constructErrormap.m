function plotSchematic_constructErrormap(environment,agent,trial,plotParams)


%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
planner = agent.planner;

arena   = environment.arena;


% place single anchor at target location, generate planned trajectory
xA = trial.target.xCenters(1);
yA = trial.target.yCenters(1);

[anchors.thCoords,anchors.rCoords] = cart2pol(xA,yA);
anchors.N = numel(anchors.rCoords);
anchors.thTol = planner.thTol_shift;
anchors.rTol  = planner.rTol_shift;

plannedTrajectory = optimizeTrajectory(anchors,belief,planner,trial,1);
plannedLikelihood = getTargetLikelihood(plannedTrajectory,belief);

% generate executed trajectory around obstacle
executedTrajectory = executeTrajectory(plannedTrajectory,planner,trial,1);
executedLikelihood = getTargetLikelihood(executedTrajectory,belief);


figure;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.4,.8]);

ax1 = subplot(3,2,1);
plotBelief(plannedLikelihood(:,:,1),belief,plotParams,'planned likelihood');hold on;
plotTrajectory(plannedTrajectory,arena,planner,trial,1,plotParams,'planned','polar');
colormap(ax1,parula)

subplot(3,2,2);
plotTrajectory(plannedTrajectory,arena,planner,trial,1,plotParams,'planned','cart');

ax2 = subplot(3,2,3);
plotBelief(executedLikelihood(:,:,1),belief,plotParams,'executed likelihood');
plotTrajectory(executedTrajectory,arena,planner,trial,1,plotParams,'executed','polar');
colormap(ax2,parula)

subplot(3,2,4);
plotTrajectory(executedTrajectory,arena,planner,trial,1,plotParams,'executed','cart');

ax3 = subplot(3,2,5);
errormap = executedLikelihood(:,:,1) - plannedLikelihood(:,:,1);
plotBelief(errormap,belief,plotParams,'errormap');
colormap(ax3,redblue)
crange = max(abs(errormap(:)));
if crange>0
    clim([-crange,crange]);
end