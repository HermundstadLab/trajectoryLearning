function trial = generateTrialStructure(arena,target,obstacle,planner,trialParams)
% GENERATETRIALSTRUCTURE Generate a structure that specifies the current
% trial configuration.
%
%   trial = GENERATETRIALSTRUCTURE(arena,target,obstacle,trialParams) takes 
%   as inputs structures that specify parameter values for the trial 
%   protocol, together with structures that specify properties of the arena,
%   target, and obstacle, and returns a structure that specifies the
%   protocol for individual trials.
%
%   In order to construct the required input structures, the functions
%   loadEnvironmentParams, generateEnvironment, loadAgentParams, and
%   loadTrialParams must be run before running loadAgentParams.
%
%   See also: LOADENVIRONMENTPARAMS, GENERATEENVIRONMENT, LOADAGENTPARAMS,
%               LOADTRIALPARAMS

%---------------------- target arrangement -------------------------------%
if strcmp(trialParams.targetArrangement,'radial')
    % define N targets arranged radially around home port
    thcTarget = linspace(0,pi,trialParams.nTargets+2);                  % define targets on semi-circle
    thcTarget([1,trialParams.nTargets+2])=[];                           % remove edges
    [~,isort] = sort(abs(thcTarget-pi/2),'descend');                    % sort so that targets begin at center 
    thcTarget = thcTarget([isort(end),isort(1:end-1)]);                 %   and alternate around center

    rcTarget  = arena.size(2)/2.*ones(1,trialParams.nTargets);          % radial distance to target centers
    [xcTarget,ycTarget] = pol2cart(thcTarget,rcTarget);                 % x,y coordinates of target centers 

elseif strcmp(trialParams.targetArrangement,'random')
    % define N targets arranged randomly around home port
    xcTarget = randu(arena.xMin,arena.xMax,trialParams.nTargets);       % x coords of target centers
    ycTarget = randu(arena.yMin,arena.yMax,trialParams.nTargets);       % y coords of target centers

else
    error('unrecoginized target arrangement');
end

%-------------------- obstacle arrangement -------------------------------%
if strcmp(trialParams.obstacleArrangement,'none')
    % no obstacles
    xcObstacle = nan(1,trialParams.nBlocks);
    ycObstacle = nan(1,trialParams.nBlocks);

elseif strcmp(trialParams.obstacleArrangement,'centered')
    % define N obstacles centered above home port
    xcObstacle = zeros(1,trialParams.nObstacles);                       % x coords of obstacle centers
    ycObstacle = 2.5.*ones(1,trialParams.nObstacles);                   % y coords of obstacle centers

elseif strcmp(trialParams.obstacleArrangement,'random')
    % define N obstacles arranged randomly around home port
    xcObstacle = randu(arena.xMin,arena.xMax,trialParams.nObstacles);   % x coords of obstacle centers
    ycObstacle = randu(arena.yMin,arena.yMax,trialParams.nObstacles);   % y coords of obstacle centers

else
    error('unrecoginized obstacle arrangement');
end


%------------------------ block ID, masks --------------------------------%
blockIDs        = [];
targetMasks     = [];
obstacleMasks   = [];
xBoundsTarget   = [];
yBoundsTarget   = [];
xBoundaryTarget = [];
yBoundaryTarget = [];
xBoundsObst     = [];
yBoundsObst     = [];
xBoundaryObst   = [];
yBoundaryObst   = [];
hBoundaryObst   = [];
boundaryCorners = [];
boundarySides   = cell(numel(xcTarget),1);

for i=1:numel(xcTarget)

    blockIDs = [blockIDs,i.*ones(1,trialParams.nTrialsPerTarget)];

    targetMaskTmp   = generateMask(arena.X,arena.Y,...
        xcTarget(i),ycTarget(i),target.width,target.height);
    targetMasks     = cat(3,targetMasks,targetMaskTmp);

    % define bounds of target
    xbTarget = [xcTarget(i)-target.width/2, xcTarget(i)+target.width/2 ];
    ybTarget = [ycTarget(i)-target.height/2,ycTarget(i)+target.height/2];

    % use bounds to extract full boundary around target
    [xbndryTarget,ybndryTarget] = getBoundary(xbTarget,ybTarget,arena.np);

    % store bounds and boundaries
    xBoundsTarget   = [xBoundsTarget;  xbTarget];
    yBoundsTarget   = [yBoundsTarget;  ybTarget];
    xBoundaryTarget = [xBoundaryTarget;xbndryTarget];
    yBoundaryTarget = [yBoundaryTarget;ybndryTarget];

    if ~strcmp(trialParams.obstacleArrangement,'none')
        obstacleMaskTmp = generateMask(arena.X,arena.Y,...
            xcObstacle(i),ycObstacle(i),obstacle.width,obstacle.height);
        obstacleMasks   = cat(3,obstacleMasks,obstacleMaskTmp);
    
        % define bounds of target
        xbObst = [xcTarget(i)-target.width/2, xcTarget(i)+target.width/2 ];
        ybObst = [ycTarget(i)-target.height/2,ycTarget(i)+target.height/2];

        % use bounds to extract full boundary (densely-sampled) around 
        % obstacle; this will be used for planning constant-velocity 
        % trajectories around obstacle
        [xbndryObst,ybndryObst,hbObst,indSides,indCorners] = ...
            getBoundary(xbObst,ybObst,planner.nxObstacle);
    
        % store bounds and boundaries
        xBoundsObst      = [xBoundsObst;    xbObst    ];
        yBoundsObst      = [yBoundsObst;    ybObst    ];
        xBoundaryObst    = [xBoundaryObst;  xbndryObst];
        yBoundaryObst    = [yBoundaryObst;  ybndryObst];
        hBoundaryObst    = [hBoundaryObst;  hbObst    ];
        boundaryCorners  = [boundaryCorners;indCorners];
        boundarySides{i} = indSides;
    else
        xBoundsObst      = [xBoundsObst;    nan(1,2)];
        yBoundsObst      = [yBoundsObst;    nan(1,2)];
        xBoundaryObst    = [xBoundaryObst;  nan(1,planner.nxObstacle)];
        yBoundaryObst    = [yBoundaryObst;  nan(1,planner.nxObstacle)];
        hBoundaryObst    = [hBoundaryObst;  nan(1,planner.nxObstacle)];
        boundaryCorners  = [boundaryCorners;nan(1,5)];
        boundarySides{i} = NaN;
    end

end

%---------------------- build output structure ----------------------------%
trial.exptType            = trialParams.exptType;
trial.targetArrangement   = trialParams.targetArrangement;
trial.obstacleArrangement = trialParams.obstacleArrangement;

trial.nTargets            = trialParams.nTargets;
trial.nObstacles          = trialParams.nObstacles;
trial.nTrials             = numel(blockIDs);
trial.blockIDs            = blockIDs;

trial.target.xCenters     = xcTarget;
trial.target.yCenters     = ycTarget;
trial.target.width        = target.width;
trial.target.height       = target.height;
trial.target.masks        = targetMasks;
trial.target.xBounds      = xBoundsTarget;
trial.target.yBounds      = yBoundsTarget;
trial.target.xBoundary    = xBoundaryTarget;
trial.target.yBoundary    = yBoundaryTarget;

trial.obstacle.xCenters   = xcObstacle;
trial.obstacle.yCenters   = ycObstacle;
trial.obstacle.width      = obstacle.width;
trial.obstacle.height     = obstacle.height;
trial.obstacle.masks      = obstacleMasks;
trial.obstacle.xBounds    = xBoundsObst;
trial.obstacle.yBounds    = yBoundsObst;
trial.obstacle.xBoundary  = xBoundaryObst;
trial.obstacle.yBoundary  = yBoundaryObst;
trial.obstacle.heading    = hBoundaryObst;
trial.obstacle.indSides   = boundarySides;
trial.obstacle.indCorners = boundaryCorners;



if ~strcmp(trialParams.obstacleArrangement,'none')
    % define bounds of four regions around obstacle, use for computing solid
    % angle of obstacle wrt anchor points
    for i=1:numel(xcTarget)
    
        % region below obstacle
        trial.obstacle.block(i).region(1).xBounds = arena.xBounds;
        trial.obstacle.block(i).region(1).yBounds = [-inf,trial.obstacle.yBounds(1)];
    
        % region to the right of the obstacle
        trial.obstacle.block(i).region(2).xBounds = [trial.obstacle.xBounds(2),inf];
        trial.obstacle.block(i).region(2).yBounds = arena.yBounds;
    
        % region above the obstacle
        trial.obstacle.block(i).region(3).xBounds = arena.xBounds;
        trial.obstacle.block(i).region(3).yBounds = [trial.obstacle.yBounds(2),inf];
        
        % region to the left of the obstacle
        trial.obstacle.block(i).region(4).xBounds = [-inf,trial.obstacle.xBounds(1)];
        trial.obstacle.block(i).region(4).yBounds = arena.yBounds;
    
        for j=1:4
            inds = indSides{j};
            trial.obstacle.block(i).region(j).xBoundary = xBoundaryObst(inds);
            trial.obstacle.block(i).region(j).yBoundary = yBoundaryObst(inds); 
        end
    
    end
end
            

end

function mask = generateMask(X,Y,xc,yc,width,height)
mask = zeros(size(X));
if ~isnan(xc) && ~isnan(yc)
    mask(X>xc-width/2 & X<xc+width/2 ...
        & Y>yc-height/2 & Y<yc+height/2) = 1; 
end
end