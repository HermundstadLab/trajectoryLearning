function [arena,belief,target,obstacle] = generateEnvironment(arenaParams,targetParams,obstacleParams)
% GENERATEENVIRONMENT Generate structures that specify the current
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


% define arena boundaries in cartesian coordinates
xBounds = [-arenaParams.width/2,arenaParams.width/2];                   % horizontal boundaries of arena
yBounds = [ 0,arenaParams.aspectRatio*arenaParams.width];               % vertical boundaries of arena
xAxes   = linspace(xBounds(1),xBounds(2),arenaParams.np);               % linear x coordinates
yAxes   = linspace(yBounds(1),yBounds(2),arenaParams.np);               % linear y coordinates
[X,Y]   = meshgrid(xAxes,yAxes);                                        % dense coordinates
arenaSize = [diff(xBounds),diff(yBounds)];                              % [width, height] of arena
[xBoundary,yBoundary] = getBoundary(xBounds,yBounds,arenaParams.np);    % define bounds of arena (for plotting)

%convert arena boundaries to polar coordinates for belief computation
thBounds = [atan2(yBounds(1),xBounds(2)),atan2(yBounds(1),xBounds(1))]; % angular boundaries of arena
rBounds  = [0,sqrt(yBounds(2).^2 + xBounds(2).^2)];                     % angular and radial boundaries of arena
thAxes   = linspace(thBounds(1),thBounds(2),arenaParams.np);            % linear theta coordinates
rAxes    = linspace(rBounds( 1), rBounds(2),arenaParams.np);            % linear r coordinates
[thNorm,rNorm] = meshgrid(linspace(0,1,arenaParams.np),...
    linspace(0,1,arenaParams.np));                                      % dense normalized coordinates
arenaSizePolar = [diff(thBounds),diff(rBounds)];                        % [angular span, radial span] of arena      
[thBoundary,rBoundary] = cart2pol(xBoundary,yBoundary);                 % define bounds of arena (for plotting)
thBoundary = wrapTo2Pi(thBoundary);

% define properties of target
targetWidth  = targetParams.relativeWidth*arenaParams.width;            % height of target
targetHeight = targetWidth.*targetParams.aspectRatio;                   % width of target

% define properties of obstacle
obstacleWidth  = obstacleParams.relativeWidth*arenaParams.width;        % height of obstacle
obstacleHeight = obstacleWidth.*obstacleParams.aspectRatio;             % width of obstacle

% store results
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

belief.size       = arenaSizePolar;
belief.np         = arenaParams.np;
belief.thAxes     = thAxes;
belief.rAxes      = rAxes;
belief.thMin      = min(thBounds);
belief.thMax      = max(thBounds);
belief.rMin       = min(rBounds);
belief.rMax       = max(rBounds);
belief.thNorm     = thNorm;
belief.rNorm      = rNorm;
belief.thBounds   = [belief.thMin,belief.thMax];
belief.rBounds    = [belief.rMin, belief.rMax];
belief.thBoundary = thBoundary;
belief.rBoundary  = rBoundary;

target.width    = targetWidth;
target.height   = targetHeight;

obstacle.width  = obstacleWidth;
obstacle.height = obstacleHeight;

end