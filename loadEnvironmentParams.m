function [arenaParams,targetParams,obstacleParams] = loadEnvironmentParams(envType,varargin)
% LOADENVIRONMENTPARAMS Loads parameters that specify environment
% properties.
%
%   [arenaParams,targetParams,obstacleParams] = LOADENVIRONMENTPARAMS(envType)
%   takes as input a string that specifies the environment type (currently 
%   configured only for a 'default' environment), and returns structures 
%   that specify parameter settings for the arena, target, and obstacles.  
%
%   See also: LOADAGENTPARAMS, LOADTRIALPARAMS

p = inputParser;

if strcmp(envType,'default')
    %generate rectangular arena, target, and obstacle

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
obstacleParams.aspectRatio   = p.Results.targetAspectRatio;