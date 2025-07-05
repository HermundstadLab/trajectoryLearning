function agentParams = loadAgentParams(agentType)
% LOADAGENTPARAMS Loads parameters that specify agent properties.
%
%   agentParams = LOADAGENTPARAMS(agentType) takes as input a string that 
%   specifies the agent type (currently configuredonly for a 'default' 
%   agent), and returns structures that specify parameter settings that 
%   will be used to construct the agent's belief, sampler, and planner 
%   modules. 
%   
%   See also: LOADENVIRONMENTPARAMS, LOADTRIALPARAMS

% NOTE:
% many of the following parameters are defined in normalized units (n.u.) 
% of the posterior discretization; others are defined in arbitrary units 
% of arena size (a.u.)

if strcmp(agentType,'default')

    agentParams.relativeWidth         = 0.05;                   % width of agent, relative to arena (a.u.)
    agentParams.likelihoodRange       = 0.9;                    % max range of likelihood (btw 0 and 1)    
    agentParams.likelihoodSigma       = 0.075;                  % SD of gaussian likelihood, expressed as percentage of belief space (n.u.) 
    
    agentParams.memoryDecay           = 0.01;                   % amount of posterior memory decay (measured as weighting to apply to uniform prior)
    agentParams.uniformPriorThreshold = 0.001;                  % threshold for detecting uniform prior (measured as the fraction DKL(prior||uniform)/DKL(delta||uniform) )
    agentParams.surpriseThreshold     = 5;                      % threshold for caching posterior (outcome is 10x more surprising under current prior compared to uniform prior)
    agentParams.resetWindow           = 2;                      % number of successive timepoints for which surprise signal must exceed threshold
    agentParams.resetFlag             = true;                   % determines whether to reset belief
                                                                % options: true/false

    agentParams.cacheSamplingMethod   = 'prop';                 % method to use to sample context from cache:
                                                                % options: 'MAP' (maximum a posteriori)
                                                                %          'prop' (proportional sampling)
                                                                %          'avg' (posterior average)
    agentParams.cacheSize             = 5;                      % maximum number of allowed items in cache (including working belief)                              
    agentParams.cacheFlag             = false;                  % determines whether agent can cache current belief
                                                                % options: true/false                                                      

    agentParams.anchorTolMerge        = 0.05;                   % tolerance for merging anchors (n.u.)
    agentParams.anchorTolShift        = 0.005;                  % tolerance for shifting anchors (n.u.)    
    agentParams.anchorSamplingNoise   = 0.01;                   % noise in sampling anchors (n.u.) 
                                                                %   (should be ~less than the ratio of the target width to arena width)
    agentParams.anchorInit            = 10;                     % initial number of anchors
    agentParams.anchorTolScaling      = true;                   % determines whether to scale tolerances around individual anchors 
                                                                % options: true (scale tolerances based on width of posterior peaks)
                                                                %          false (used fixed tolerance for all anchors)
    agentParams.anchorOrderMethod     = 'TSP';                  % type of ordering to use for anchor points; 
                                                                % options: 'TSP' (solves approx traveling salesman)
                                                                %          'angle' (sorts anchors based on angle)

    agentParams.errorThreshold        = -0.05;                  % error threshold for augmenting anchors points
    
    agentParams.timeInterp            = 100;                    % number of timepoints to use to interpolate trajectories
    agentParams.spaceInterp           = 0.25;                   % number of spatial points to interpolate trajectories around obstacles
                                                                %   (expressed as percentage of belief space per unit distance)

    agentParams.nOptima               = 5;                      % number of optimzations to perform for initial heading

%elseif strcmp(agentType,'new agent type')                      % uncomment to add new agent types
else
    error('unrecognized agent type')
end

end



