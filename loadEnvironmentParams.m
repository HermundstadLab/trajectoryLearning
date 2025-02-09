function [arenaParams,targetParams,obstacleParams] = loadEnvironmentParams(envType)
% LOADENVIRONMENTPARAMS Loads parameters that specify environment
% properties.
%
%   [arenaParams,targetParams,obstacleParams] = LOADENVIRONMENTPARAMS(envType)
%   takes as input a string that specifies the environment type (currently 
%   configured only for a 'default' environment), and returns structures 
%   that specify parameter settings for the arena, target, and obstacles.  
%
%   See also: LOADAGENTPARAMS, LOADTRIALPARAMS

if strcmp(envType,'default')
    %generate rectangular arena, target, and obstacle

    arenaParams.width            = 10;          % width of arena
    arenaParams.aspectRatio      = 1;           % aspect ratio of arena (height relative to width)
    arenaParams.np               = 100;         % number of points to use in discretizing the environment
                                                % (used in computing beliefs & plotting arena)

    targetParams.relativeWidth   = 0.1;         % width of target, relative to arena
    targetParams.aspectRatio     = 1;           % aspect ratio of target (height relative to width)

    obstacleParams.relativeWidth = 0.4;         % width of obstacle, relative to arena
    obstacleParams.aspectRatio   = 0.1;         % aspect ratio of obstacle (height relative to width)

%elseif strcmp(envType,'new env type')          % uncomment to add new environments
else
    error('unrecognized environment type')
end