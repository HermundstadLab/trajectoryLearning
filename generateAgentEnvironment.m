function [arena,belief,sampler,planner,target,obstacle] = generateAgentEnvironment(arenaParams,targetParams,obstacleParams,agentParams)
% GENERATEAGENTENVIRONMENT Generate structures that specify the current
% environment for simulations.
%
%   [arena,belief,target,obstacle] = GENERATEENVIRONMENT(arenaParams,...
%   targetParams,obstacleParams) takes as inputs several structures that 
%   specify parameter values, and returns structures that specify the
%   settings for the arena, belief, target, and obstacles.
%
%   In order to construct the required input structures, the function
%   loadEnvironmentParams must be run before running loadAgentParams.
%
%   See also: LOADENVIRONMENTPARAMS, GENERATETRIALSTRUCTURE


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                OVERVIEW                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This setting assumes that the agent knows the overall size of the arena,
% which it could infer by running a trajectory along the boundary of the
% arena. We assume that it uses this knowledge to specify the size of its 
% internal belief space. In running such a boundary trajectory, we assume 
% that the agent's perception of the arena is slightly smaller than it's
% actual size, due to the finite size of the agent itself; this constrains
% the belief space to be slightly smaller than the true arena size. Below,
% we differentiate between the true arena size (in cartesian coordinates),
% the perceived arena size (in cartesian coordinates), and the size of the
% belief space (in polar coordinates).

%------------------------ define agent size ------------------------------%
agentWidth = agentParams.relativeWidth.*arenaParams.width;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                    ARENA SPACE (CARTESIAN COORDINATES)                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-------------------- define absolute arena bounds -----------------------%

xBounds = [-arenaParams.width/2,arenaParams.width/2];                   % horizontal boundaries of arena
yBounds = [ 0,arenaParams.aspectRatio*arenaParams.width];               % vertical boundaries of arena
yBounds = yBounds-agentWidth/2;                                         % shift downward so that agent's arena bounds
                                                                        %   are aligned with y = 0
xAxes   = linspace(xBounds(1),xBounds(2),arenaParams.np);               % linear x coordinates
yAxes   = linspace(yBounds(1),yBounds(2),arenaParams.np);               % linear y coordinates
[X,Y]   = meshgrid(xAxes,yAxes);                                        % dense coordinates
arenaSize = [diff(xBounds),diff(yBounds)];                              % [width, height] of arena
[xBoundary,yBoundary] = getBoundary(xBounds,yBounds,4*arenaParams.np);  % define bounds of arena (for plotting)


%---------- define relative arena bounds from agent's perspective --------%

xBounds_agent = xBounds + agentWidth/2*[1,-1];
yBounds_agent = yBounds + agentWidth/2*[1,-1];
[xBoundary_agent,yBoundary_agent,heading_agent,indSides_agent,...
    indCorners_agent,corners_agent] = getBoundary(xBounds_agent,yBounds_agent,4*arenaParams.np);

%------------------ define properties of target and obstacle -------------%

% define properties of target
targetWidth  = targetParams.relativeWidth*arenaParams.width;            % height of target
targetHeight = targetWidth.*targetParams.aspectRatio;                   % width of target

% define properties of obstacle
obstacleWidth  = obstacleParams.relativeWidth*arenaParams.width;        % height of obstacle
obstacleHeight = obstacleWidth.*obstacleParams.aspectRatio;             % width of obstacle


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                      BELIEF SPACE (POLAR COORDINATES)                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--------------------- define axes for belief computation ----------------%

thBounds = [atan2(yBounds_agent(1),xBounds_agent(2)),...                % convert arena boundaries to polar coordinates
    atan2(yBounds_agent(1),xBounds_agent(1))];                          % angular boundaries of arena
rBounds  = [0,sqrt(yBounds_agent(2).^2 + xBounds_agent(2).^2)];         % angular and radial boundaries of arena
thAxes   = linspace(thBounds(1),thBounds(2),arenaParams.np);            % linear theta coordinates
rAxes    = linspace(rBounds( 1), rBounds(2),arenaParams.np);            % linear r coordinates
[thNorm,rNorm] = meshgrid(linspace(0,1,arenaParams.np),...
    linspace(0,1,arenaParams.np));                                      % dense normalized coordinates
arenaSizePolar = [diff(thBounds),diff(rBounds)];                        % [angular span, radial span] of arena      
[thBoundary,rBoundary] = cart2pol(xBoundary_agent,yBoundary_agent);     % define bounds of arena (for plotting)
thBoundary = wrapTo2Pi(thBoundary);


%------------------------ build arena structure --------------------------%

arena.size      = arenaSize;
arena.np        = arenaParams.np;
arena.xAxes     = xAxes;
arena.yAxes     = yAxes;
arena.X         = X;
arena.Y         = Y;
arena.xMin      = min(xBounds);
arena.xMax      = max(xBounds);
arena.yMin      = min(yBounds);
arena.yMax      = max(yBounds);
arena.xBounds   = xBounds;
arena.yBounds   = yBounds;
arena.xBoundary = xBoundary;
arena.yBoundary = yBoundary;

arena.agent.xBounds    = xBounds_agent;
arena.agent.yBounds    = yBounds_agent;
arena.agent.xBoundary  = xBoundary_agent;
arena.agent.yBoundary  = yBoundary_agent;
arena.agent.heading    = heading_agent;
arena.agent.indSides   = indSides_agent;
arena.agent.indCorners = indCorners_agent;
arena.agent.corners    = corners_agent;


%-------------- build target and obstacle structures ---------------------%

target.width    = targetWidth;
target.height   = targetHeight;

obstacle.width  = obstacleWidth;
obstacle.height = obstacleHeight;

%----------------------- build belief structure --------------------------%

belief.size           = arenaSizePolar;
belief.np             = arenaParams.np;
belief.thAxes         = thAxes;
belief.rAxes          = rAxes;
belief.thMin          = min(thBounds);
belief.thMax          = max(thBounds);
belief.rMin           = min(rBounds);
belief.rMax           = max(rBounds);
belief.thNorm         = thNorm;
belief.rNorm          = rNorm;
belief.thBounds       = [belief.thMin,belief.thMax];
belief.rBounds        = [belief.rMin, belief.rMax];
belief.thBoundary     = thBoundary;
belief.rBoundary      = rBoundary;

belief.rangeL         = agentParams.likelihoodRange;                % max range of likelihood (btw 0 and 1)    
belief.sigmaL         = agentParams.likelihoodSigma*belief.np;      % SD of gaussian likelihood (n.u.) 

belief.npExclude      = agentParams.anchorTolMerge.*belief.np;      % radial distance to exclude around home port (n.u.)
belief.rMinAnchors    = belief.rMin + ...                           % define minimum radius for anchors selected beyond home port
    agentParams.anchorTolMerge*(belief.rMax-belief.rMin);
belief.cacheThreshold = agentParams.cacheThreshold;                 % surprise threshold for caching posterior;
belief.cacheWindow    = agentParams.cacheWindow;                    % number of successive timepoints that cache signal must exceed threshold
belief.mask           = createPosteriorMask(belief);                % create posterior mask based on arena bounds
belief.cache          = agentParams.cacheFlag;                      % determines whether to cache posterior
belief.boundaryTol    = 1./belief.np;                               % tolerance for determining boundary anchors


%---------------------- build sampler structure --------------------------%

sampler.minPeakDist    = agentParams.anchorTolMerge.*belief.np;     % min distance between peaks in posterior (n.u.)
sampler.minPeakHeight  = 1./(belief.np.^2);                         % min height of peaks in posterior (n.u.)
sampler.nAnchorsMax    = agentParams.anchorMax;                     % maximum number of anchors
sampler.errorThreshold = agentParams.errorThreshold;                % error threshold for augmenting anchors points


%--------------------- build planner structure ---------------------------%

planner.agentWidth     = agentWidth;                                % width of agent
planner.nInterp        = agentParams.timeInterp;                    % number of timepoints to use to interpolate trajectories
planner.dt             = 1/planner.nInterp;                         % time discretization
planner.rScale         = belief.size(2)./2;                         % used to scale the execution time of trajectory segments
                                                                    %   (defined in units of distance per time)  
planner.tol_shift      = agentParams.anchorTolShift;                % default tolerance for shifting anchors                                                                    
planner.thTol_shift    = agentParams.anchorTolShift*belief.size(1); % default angular tolerance for shifting anchors (a.u.)
planner.rTol_shift     = agentParams.anchorTolShift*belief.size(2); % default radial tolerance for shifting anchors (a.u.)
planner.scaleTol       = agentParams.anchorTolScaling;              % determines whether to scale tolerances around individual anchors 
planner.orderType      = agentParams.anchorOrderMethod;             % type of ordering to use for anchor points; 


%----------------- define and store boundary trajectory ------------------%

% define anchor points at corners of arena
xCoords = [0,max(xBounds_agent),max(xBounds_agent),min(xBounds_agent),min(xBounds_agent),0];
yCoords = [min(yBounds_agent),min(yBounds_agent),max(yBounds_agent),max(yBounds_agent),...
    min(yBounds_agent),min(yBounds_agent)];

[anchors.thCoords,anchors.rCoords] = cart2pol(xCoords,yCoords);
anchors.N = numel(anchors.thCoords);

planner.boundary.anchors = anchors;
planner.boundary.xBounds = xBounds_agent;
planner.boundary.yBounds = yBounds_agent;

end

function mask = createPosteriorMask(belief)

mask = nan(belief.np);
for i=1:numel(belief.thBoundary)
    [~,ii] = min((belief.thBoundary(i)-belief.thAxes).^2);
    [~,jj] = min((belief.rBoundary( i)-belief.rAxes ).^2);
    mask(belief.npExclude+1:jj,ii) = 1;
end

% check that the mask is symmetric
m = abs(mask-fliplr(mask));
if any(m(:)>0)
    error('posterior mask is not symmetric')
end

end