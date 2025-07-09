function plotSchematic_planTrajectory(singleAgentResults,environment,agent,trial,trialID,plotParams)

%---------------------- extract agent structures -------------------------%
belief  = agent.belief;
sampler = agent.sampler;
planner = agent.planner;

arena   = environment.arena;


executedTrajectory  = singleAgentResults.trajectory.executed.path{trialID};

figure;hold on;
set(gcf,'color','w','units','normalized','Position',[.025,.025,.4,.95])

xAnchors = arena.xBounds(1)+[.2,.8]*diff(arena.xBounds);
yAnchors = arena.yBounds(1)+[.2,.8]*diff(arena.yBounds);
[anchors.thCoords,anchors.rCoords] = cart2pol(xAnchors,yAnchors);
anchors.N = 2;

trajectorySegment = planTrajectory(anchors,pi/4,planner);


% plot trajectory segment in cartesian coordinates
subplot(4,2,1);
plotTrajectory(trajectorySegment,arena,planner,trial,trialID,plotParams,'executed','cart');
hold on; plotAnchors(trajectorySegment,arena,belief,sampler,planner,plotParams,'cart');

% plot full trajectory in cartesian coordinates
subplot(4,2,2);
plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'executed','cart');
hold on; plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'cart');

% plot trajectory segment in polar coordinates
subplot(4,2,3);
plot(belief.thBoundary,belief.rBoundary,'color',plotParams.cBoundary,'linewidth',plotParams.lw);
hold on; plotAnchors(trajectorySegment,arena,belief,sampler,planner,plotParams,'polar');
plotTrajectory(trajectorySegment,arena,planner,trial,trialID,plotParams,'planned','polar');

% plot full trajectory in polar coordinates
subplot(4,2,4);
plot(belief.thBoundary,belief.rBoundary,'color',plotParams.cBoundary,'linewidth',plotParams.lw);
hold on; plotAnchors(executedTrajectory,arena,belief,sampler,planner,plotParams,'polar');
plotTrajectory(executedTrajectory,arena,planner,trial,trialID,plotParams,'planned','polar');

% plot control params for trajectory segment
subplot(4,2,5); plotControlParams(trajectorySegment,planner,plotParams,'planned','velocity');title('control params')
subplot(4,2,7); plotControlParams(trajectorySegment,planner,plotParams,'planned','heading');

% plot control params for full trajectory
subplot(4,2,6); plotControlParams(executedTrajectory,planner,plotParams,'planned','velocity');title('control params')
subplot(4,2,8); plotControlParams(executedTrajectory,planner,plotParams,'planned','heading');