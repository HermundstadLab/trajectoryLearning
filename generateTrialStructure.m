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
%   See also: LOADENVIRONMENTPARAMS, GENERATEAGENTENVIRONMENT, 
%             LOADAGENTPARAMS, LOADTRIALPARAMS

%---------------------- target arrangement -------------------------------%
if strcmp(trialParams.targetArrangement,'radial')
    % define N targets arranged radially around home port
    thcTarget = linspace(0,pi,trialParams.nTargets+4);                  % define targets on semi-circle
    thcTarget([1,2,trialParams.nTargets+3,trialParams.nTargets+4])=[];  % remove edges
    [~,isort] = sort(abs(thcTarget-pi/2),'descend');                    % sort so that targets begin at center 
    thcTarget = thcTarget([isort(end),isort(1:end-1)]);                 %   and alternate around center

    rcTarget  = arena.size(2)/2.*ones(1,trialParams.nTargets);          % radial distance to target centers
    [xcTarget,ycTarget] = pol2cart(thcTarget,rcTarget);                 % x,y coordinates of target centers 

elseif strcmp(trialParams.targetArrangement,'random')
    % define N targets arranged randomly around home port
    xcTarget = randu(arena.xMin,arena.xMax,trialParams.nTargets);       % x coords of target centers
    ycTarget = randu(arena.yMin,arena.yMax,trialParams.nTargets);       % y coords of target centers

elseif strcmp(trialParams.targetArrangement,'centered')
    % define N targets centered above home port
    xcTarget = zeros(1,trialParams.nTargets);                           % x coords of target centers
    ycTarget = arena.size(2)/2.*ones(1,trialParams.nTargets);           % y coords of target centers

else
    error('unrecoginized target arrangement');
end

%-------------------- obstacle arrangement -------------------------------%
if strcmp(trialParams.obstacleArrangement,'none')
    % no obstacles
    xcObstacle = nan(1,trialParams.nBlocks);
    ycObstacle = nan(1,trialParams.nBlocks);

elseif strcmp(trialParams.obstacleArrangement,'centered')
    % define obstacles centered above home port
    xcObstacle = zeros(1, trialParams.nBlocks);                         % x coords of obstacle centers
    ycObstacle = 2.5.*ones(1, trialParams.nBlocks);                     % y coords of obstacle centers

    xcObstacle(trialParams.obstacleTrials<1) = nan;
    ycObstacle(trialParams.obstacleTrials<1) = nan;

elseif strcmp(trialParams.obstacleArrangement,'random')
    % define N obstacles arranged randomly around home port
    xcObstacle = randu(arena.xMin,arena.xMax,trialParams.nBlocks);      % x coords of obstacle centers
    ycObstacle = randu(arena.yMin,arena.yMax,trialParams.nBlocks);      % y coords of obstacle centers

    xcObstacle(trialParams.obstacleTrials<1) = nan;
    ycObstacle(trialParams.obstacleTrials<1) = nan;

else
    error('unrecoginized obstacle arrangement');
end


%----------------- extract agent's view of arena boundary ----------------%
trial.arena.agent     = arena.agent;
trial.arena.xBounds   = arena.xBounds;
trial.arena.yBounds   = arena.yBounds;
trial.arena.xBoundary = arena.xBoundary;
trial.arena.yBoundary = arena.yBoundary;

for i=1:4
    trial.arena.agent.block(1).region(i).xBounds = arena.xBounds;
    trial.arena.agent.block(1).region(i).yBounds = arena.yBounds;

    inds = trial.arena.agent.indSides{i};
    trial.arena.agent.block(1).region(i).xBoundary = trial.arena.agent.xBoundary(inds);
    trial.arena.agent.block(1).region(i).yBoundary = trial.arena.agent.yBoundary(inds); 
end



%------------- initialize variables for targets & obstacles --------------%

% initialize variables for target
blockIDs        = [];
targetMasks     = [];
obstacleMasks   = [];
xBoundsTarget   = [];
yBoundsTarget   = [];
xBoundaryTarget = [];
yBoundaryTarget = [];

% initialize variables for obstacle
xBoundsObst   = [];
yBoundsObst   = [];
xBoundaryObst = [];
yBoundaryObst = [];

% initial variables for agent's view of obstacle
xBoundsObst_agent       = [];
yBoundsObst_agent       = [];
xBoundaryObst_agent     = [];
yBoundaryObst_agent     = [];
hBoundaryObst_agent     = [];
boundaryCornerIDs_agent = [];
boundaryCorners_agent   = [];
boundarySides_agent     = cell(numel(xcTarget),1);

for i=1:numel(xcTarget)

    %----------------------- define target bounds ------------------------%
    blockIDs = [blockIDs,i.*ones(1,trialParams.nTrialsPerTarget)];

    targetMaskTmp   = generateMask(arena.X,arena.Y,...
        xcTarget(i),ycTarget(i),target.width,target.height);
    targetMasks     = cat(3,targetMasks,targetMaskTmp);

    % define bounds of target
    xbTarget = [xcTarget(i)-target.width/2, xcTarget(i)+target.width/2 ];
    ybTarget = [ycTarget(i)-target.height/2,ycTarget(i)+target.height/2];

    % use bounds to extract full boundary around target
    [xbndryTarget,ybndryTarget] = getBoundary(xbTarget,ybTarget,4*arena.np);

    % store bounds and boundaries
    xBoundsTarget   = [xBoundsTarget;   xbTarget    ];
    yBoundsTarget   = [yBoundsTarget;   ybTarget    ];
    xBoundaryTarget = [xBoundaryTarget; xbndryTarget];
    yBoundaryTarget = [yBoundaryTarget; ybndryTarget];

    %----------------------- define obstacle bounds ----------------------% 


    if trialParams.obstacleTrials(i)>0.5
        xcObstacleTmp = xcObstacle(i); 
        ycObstacleTmp = ycObstacle(i); 
    else
        xcObstacleTmp = 0; 
        ycObstacleTmp = 0; 
    end

    %------------- true bounds of obstacle -----------%
    % generate obstacle mask (for plotting)
    obstacleMaskTmp = generateMask(arena.X,arena.Y,...
        xcObstacleTmp,ycObstacleTmp,obstacle.width,obstacle.height);
    if trialParams.obstacleTrials(i)<0.5
        obstacleMaskTmp = nan(size(obstacleMaskTmp));
    end
    obstacleMasks   = cat(3,obstacleMasks,obstacleMaskTmp);

    % define bounds of obstacle
    xbObst = [xcObstacleTmp - obstacle.width/2, xcObstacleTmp + obstacle.width/2 ];
    ybObst = [ycObstacleTmp - obstacle.height/2,ycObstacleTmp + obstacle.height/2];

    % extract full boundary around obstacle
    [xbndryObst,ybndryObst] = ...
        getBoundary(xbObst,ybObst,4*arena.np);

    %----- effective bounds that agent traverses ----%

    % define bounds of obstacle
    xbObst_agent = [xcObstacleTmp - obstacle.width/2  - planner.agentWidth/2, xcObstacleTmp + obstacle.width/2  + planner.agentWidth/2];
    ybObst_agent = [ycObstacleTmp - obstacle.height/2 - planner.agentWidth/2, ycObstacleTmp + obstacle.height/2 + planner.agentWidth/2];

    % extract agent's view of obstacle boundary
    [xbndryObst_agent,ybndryObst_agent,hbObst_agent,indSides_agent,indCorners_agent,corners_agent] = ...
        getBoundary(xbObst_agent,ybObst_agent,4*arena.np);

    if trialParams.obstacleTrials(i)>0.5
        % store bounds and boundaries
        xBoundsObst             = [xBoundsObst;    xbObst    ];
        yBoundsObst             = [yBoundsObst;    ybObst    ];
        xBoundaryObst           = [xBoundaryObst;  xbndryObst];
        yBoundaryObst           = [yBoundaryObst;  ybndryObst];
        
        % store bounds and boundaries
        xBoundsObst_agent       = [xBoundsObst_agent;       xbObst_agent    ];
        yBoundsObst_agent       = [yBoundsObst_agent;       ybObst_agent    ];
        xBoundaryObst_agent     = [xBoundaryObst_agent;     xbndryObst_agent];
        yBoundaryObst_agent     = [yBoundaryObst_agent;     ybndryObst_agent];
        hBoundaryObst_agent     = [hBoundaryObst_agent;     hbObst_agent    ];
        boundaryCornerIDs_agent = [boundaryCornerIDs_agent; indCorners_agent];
        boundaryCorners_agent   = [boundaryCorners_agent;   corners_agent   ];
        boundarySides_agent{i}  = indSides_agent;

    else
        % store bounds and boundaries
        xBoundsObst             = [xBoundsObst;    nan(size(xbObst))    ];
        yBoundsObst             = [yBoundsObst;    nan(size(ybObst))    ];
        xBoundaryObst           = [xBoundaryObst;  nan(size(xbndryObst))];
        yBoundaryObst           = [yBoundaryObst;  nan(size(ybndryObst))];

        xBoundsObst_agent       = [xBoundsObst_agent;       nan(size(xbObst_agent))     ];
        yBoundsObst_agent       = [yBoundsObst_agent;       nan(size(ybObst_agent))     ];
        xBoundaryObst_agent     = [xBoundaryObst_agent;     nan(size(xbndryObst_agent)) ];
        yBoundaryObst_agent     = [yBoundaryObst_agent;     nan(size(ybndryObst_agent)) ];
        hBoundaryObst_agent     = [hBoundaryObst_agent;     nan(size(hbObst_agent))     ];
        boundaryCornerIDs_agent = [boundaryCornerIDs_agent; nan(size(indCorners_agent)) ];
        boundaryCorners_agent   = [boundaryCorners_agent;   nan(size(corners_agent))    ];
        boundarySides_agent{i}  = NaN;
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

trial.obstacle.agent.xBounds    = xBoundsObst_agent;
trial.obstacle.agent.yBounds    = yBoundsObst_agent;
trial.obstacle.agent.xBoundary  = xBoundaryObst_agent;
trial.obstacle.agent.yBoundary  = yBoundaryObst_agent;
trial.obstacle.agent.heading    = hBoundaryObst_agent;
trial.obstacle.agent.indSides   = boundarySides_agent;
trial.obstacle.agent.indCorners = boundaryCornerIDs_agent;
trial.obstacle.agent.corners    = boundaryCorners_agent;


for i=1:numel(xcTarget)
    if trialParams.obstacleTrials(i)>0.5
    % define bounds of four regions around obstacle, use for computing solid
    % angle of obstacle wrt anchor points
    %for i=1:numel(xcTarget)
    
        % region below obstacle
        trial.obstacle.agent.block(i).region(1).xBounds = arena.xBounds;
        trial.obstacle.agent.block(i).region(1).yBounds = [-inf,trial.obstacle.agent.yBounds(i,1)];
    
        % region to the right of the obstacle
        trial.obstacle.agent.block(i).region(2).xBounds = [trial.obstacle.agent.xBounds(i,2),inf];
        trial.obstacle.agent.block(i).region(2).yBounds = arena.yBounds;
    
        % region above the obstacle
        trial.obstacle.agent.block(i).region(3).xBounds = arena.xBounds;
        trial.obstacle.agent.block(i).region(3).yBounds = [trial.obstacle.agent.yBounds(i,2),inf];
        
        % region to the left of the obstacle
        trial.obstacle.agent.block(i).region(4).xBounds = [-inf,trial.obstacle.agent.xBounds(i,1)];
        trial.obstacle.agent.block(i).region(4).yBounds = arena.yBounds;
    
        for j=1:4
            inds = indSides_agent{j};
            trial.obstacle.agent.block(i).region(j).xBoundary = trial.obstacle.agent.xBoundary(inds);
            trial.obstacle.agent.block(i).region(j).yBoundary = trial.obstacle.agent.yBoundary(inds); 
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