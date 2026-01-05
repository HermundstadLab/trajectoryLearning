function [agent,environment] = generateEnvironment(envType,varargin)
% GENERATEENVIRONMENT Generate structures that specify the current
% environment for agent simulations.
%
%   [agent, environment] = GENERATEENVIRONMENT(envType, varargin) takes as 
%   input a string that specifies the configuration of the environment 
%   (currently only configured for a 'default' environment). This function 
%   returns a structure that specifies the environment configuration 
%   ('environment') and a second structure ('agent') that specified
%   parameters of an agent that are coupled to the structure of the
%   environment (such as the relative size of the agent, and its view of
%   the boundary around the arena. Variable input arguments can be used to
%   override default parameter settings.
%
%   REQUIRED INPUTS:
%   'envType'                   type of environment to generate ('default')
%
%   VARIABLE INPUTS:  
%   'arenaWidth':               width of arena
%   'arenaAspectRatio':         aspect ratio of arena (height relative to width)
%   'arenaNp':                  number of points to use in discretizing the environment
%   'agentWidth':               width of agent 
%   'targetRelativeWidth':      width of target, relative to arena
%   'obstacleRelativeWidth':    width of obstacle, relative to arena
%   'obstacleAspectRatio':      aspect ratio of obstacle (height relative to width)
%
%   See also: GENERATEAGENT, GENERATETRIALSTRUCTURE


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


%--------------------------- parse inputs --------------------------------%

p = inputParser;

if strcmp(envType,'default')
    %generate rectangular arena, target, and obstacle

    default_agentWidth            = 0.5;      % width of agent

    default_arenaWidth            = 10;       % width of arena
    default_arenaAspectRatio      = 1;        % aspect ratio of arena (height relative to width)
    default_arenaNp               = 100;      % number of points to use in discretizing the environment
                                              % (used in computing beliefs & plotting arena)

    default_targetRelativeWidth   = 0.1;      % width of target, relative to arena
    default_targetAspectRatio     = 1;        % aspect ratio of target (height relative to width)

    default_obstacleRelativeWidth = 0.4;      % width of obstacle, relative to arena
    default_obstacleAspectRatio   = 0.1;      % aspect ratio of obstacle (height relative to width)

%elseif strcmp(envType,'new env type')        % uncomment to add new environments
else
    error('unrecognized environment type')
end

validEnvTypes = {'default'};
checkEnvTypes = @(x) any(validatestring(x,validEnvTypes));

validateNumeric     = @(x) isnumeric(x) && isscalar(x);
validateInteger     = @(x) floor(x)==x;
validateLessThanOne = @(x) x>=0 && x<=1;

addRequired( p,'envType',checkEnvTypes);

addParameter(p,'arenaNp',default_arenaNp,validateInteger)

addParameter(p,'targetRelativeWidth',default_targetRelativeWidth,validateLessThanOne)
addParameter(p,'obstacleRelativeWidth',default_obstacleRelativeWidth,validateLessThanOne)

addParameter(p,'agentWidth',default_agentWidth,validateNumeric)
addParameter(p,'arenaWidth',default_arenaWidth,validateNumeric)
addParameter(p,'arenaAspectRatio',default_arenaAspectRatio,validateNumeric)
addParameter(p,'targetAspectRatio',default_targetAspectRatio,validateNumeric)
addParameter(p,'obstacleAspectRatio',default_obstacleAspectRatio,validateNumeric)

parse(p,envType,varargin{:})

arenaParams.width            = p.Results.arenaWidth;
arenaParams.aspectRatio      = p.Results.arenaAspectRatio;
arenaParams.np               = p.Results.arenaNp;   

targetParams.relativeWidth   = p.Results.targetRelativeWidth;
targetParams.aspectRatio     = p.Results.targetAspectRatio;

obstacleParams.relativeWidth = p.Results.obstacleRelativeWidth;
obstacleParams.aspectRatio   = p.Results.obstacleAspectRatio;

agentWidth = p.Results.agentWidth;

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
%            maps internal belief coordinates onto physical units         %

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

environment.arena.size      = arenaSize;
environment.arena.np        = arenaParams.np;
environment.arena.xAxes     = xAxes;
environment.arena.yAxes     = yAxes;
environment.arena.X         = X;
environment.arena.Y         = Y;
environment.arena.xMin      = min(xBounds);
environment.arena.xMax      = max(xBounds);
environment.arena.yMin      = min(yBounds);
environment.arena.yMax      = max(yBounds);
environment.arena.xBounds   = xBounds;
environment.arena.yBounds   = yBounds;
environment.arena.xBoundary = xBoundary;
environment.arena.yBoundary = yBoundary;

environment.arena.agent.agentWidth = agentWidth;
environment.arena.agent.xBounds    = xBounds_agent;
environment.arena.agent.yBounds    = yBounds_agent;
environment.arena.agent.xBoundary  = xBoundary_agent;
environment.arena.agent.yBoundary  = yBoundary_agent;
environment.arena.agent.heading    = heading_agent;
environment.arena.agent.indSides   = indSides_agent;
environment.arena.agent.indCorners = indCorners_agent;
environment.arena.agent.corners    = corners_agent;


%-------------- build target and obstacle structures ---------------------%

environment.target.width    = targetWidth;
environment.target.height   = targetHeight;

environment.obstacle.width  = obstacleWidth;
environment.obstacle.height = obstacleHeight;

environment.params = p.Results;

%-------------------- define boundaries of belief ------------------------%

agent.belief.size           = arenaSizePolar;
agent.belief.np             = arenaParams.np;
agent.belief.thAxes         = thAxes;
agent.belief.rAxes          = rAxes;
agent.belief.thMin          = min(thBounds);
agent.belief.thMax          = max(thBounds);
agent.belief.rMin           = min(rBounds);
agent.belief.rMax           = max(rBounds);
agent.belief.thNorm         = thNorm;
agent.belief.rNorm          = rNorm;
agent.belief.thBounds       = [agent.belief.thMin,agent.belief.thMax];
agent.belief.rBounds        = [agent.belief.rMin, agent.belief.rMax];
agent.belief.thBoundary     = thBoundary;
agent.belief.rBoundary      = rBoundary;

%----------------- define and store boundary trajectory ------------------%

% define anchor points at corners of arena
xCoords = [0,max(xBounds_agent),max(xBounds_agent),min(xBounds_agent),min(xBounds_agent),0];
yCoords = [min(yBounds_agent),min(yBounds_agent),max(yBounds_agent),max(yBounds_agent),...
    min(yBounds_agent),min(yBounds_agent)];

[anchors.thCoords,anchors.rCoords] = cart2pol(xCoords,yCoords);
anchors.N = numel(anchors.thCoords);

agent.planner.boundary.anchors = anchors;
agent.planner.boundary.xBounds = xBounds_agent;
agent.planner.boundary.yBounds = yBounds_agent;
agent.planner.agentWidth = agentWidth;

end