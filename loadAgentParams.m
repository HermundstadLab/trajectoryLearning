function agentParams = loadAgentParams(agentType,varargin)
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

p = inputParser;

if strcmp(agentType,'default')

    default_relativeWidth         = 0.05;                   % width of agent, relative to arena (a.u.)
    default_likelihoodRange       = 0.9;                    % max range of likelihood (btw 0 and 1)    
    default_likelihoodSigma       = 0.075;                  % SD of gaussian likelihood, expressed as percentage of belief space (n.u.) 
    
    default_memoryDecay           = 0.01;                   % amount of posterior memory decay (measured as weighting to apply to uniform prior)
    default_uniformPriorThreshold = 0.001;                  % threshold for detecting uniform prior (measured as the fraction DKL(prior||uniform)/DKL(delta||uniform) )
    default_surpriseThreshold     = 5;                      % threshold for caching posterior (outcome is 10x more surprising under current prior compared to uniform prior)
    default_resetWindow           = 2;                      % number of successive timepoints for which surprise signal must exceed threshold
    default_resetFlag             = true;                   % determines whether to reset belief
                                                            % options: true/false

    default_cacheSamplingMethod   = 'prop';                 % method to use to sample context from cache:
                                                            % options: 'MAP' (maximum a posteriori)
                                                            %          'prop' (proportional sampling)
                                                            %          'avg' (posterior average)
    default_cacheSize             = 5;                      % maximum number of allowed items in cache (including working belief)                              
    default_cacheFlag             = false;                  % determines whether agent can cache current belief
                                                            % options: true/false                                                      

    default_anchorTolMerge        = 0.05;                   % tolerance for merging anchors (n.u.)
    default_anchorTolShift        = 0.005;                  % tolerance for shifting anchors (n.u.)    
    default_anchorSamplingNoise   = 0.01;                   % noise in sampling anchors (n.u.) 
                                                            %   (should be ~less than the ratio of the target width to arena width)
    default_anchorInit            = 10;                     % initial number of anchors
    default_anchorTolScaling      = true;                   % determines whether to scale tolerances around individual anchors 
                                                            % options: true (scale tolerances based on width of posterior peaks)
                                                            %          false (used fixed tolerance for all anchors)
    default_anchorOrderMethod     = 'TSP';                  % type of ordering to use for anchor points; 
                                                            % options: 'TSP' (solves approx traveling salesman)
                                                            %          'angle' (sorts anchors based on angle)

    default_errorThreshold        = -0.05;                  % error threshold for augmenting anchors points
    
    default_timeInterp            = 100;                    % number of timepoints to use to interpolate trajectories
    default_spaceInterp           = 0.25;                   % number of spatial points to interpolate trajectories around obstacles
                                                            %   (expressed as percentage of belief space per unit distance)

    default_nOptima               = 5;                      % number of optimzations to perform for initial heading

%elseif strcmp(agentType,'new agent type')                  % uncomment to add new agent types
else
    error('unrecognized agent type')
end



validAgentTypes = {'default'};
checkAgentTypes = @(x) any(validatestring(x,validAgentTypes));

validCacheSamplings = {'prop','MAP','avg'};
checkCacheSamplings = @(x) any(validatestring(x,validCacheSamplings));

validAnchorOrderings = {'TSP','angle'};
checkAnchorOrderings = @(x) any(validatestring(x,validAnchorOrderings));

validateLogical     = @(x) islogical(x);
validateNegative    = @(x) isnumeric(x) && isscalar(x) && (x > 0);
validateNumeric     = @(x) isnumeric(x) && isscalar(x);
validateInteger     = @(x) floor(x)==x;
validateLessThanOne = @(x) x>=0 && x<=1;

addRequired( p,'agentType',checkAgentTypes);

addOptional( p,'cacheSamplingMethod',default_cacheSamplingMethod,checkCacheSamplings);
addOptional( p,'anchorOrderMethod',default_anchorOrderMethod,checkAnchorOrderings);

addParameter(p,'anchorTolScaling',default_anchorTolScaling,validateLogical)
addParameter(p,'resetFlag',default_resetFlag,validateLogical)
addParameter(p,'cacheFlag',default_cacheFlag,validateLogical)

addParameter(p,'resetWindow',default_resetWindow,validateInteger)
addParameter(p,'cacheSize',default_cacheSize,validateInteger)
addParameter(p,'anchorInit',default_anchorInit,validateInteger)
addParameter(p,'nOptima',default_nOptima,validateInteger)
addParameter(p,'timeInterp',default_timeInterp,validateInteger)

addParameter(p,'relativeWidth',default_relativeWidth,validateLessThanOne)
addParameter(p,'likelihoodRange',default_likelihoodRange,validateLessThanOne)

addParameter(p,'likelihoodSigma',default_likelihoodSigma,validateNumeric)
addParameter(p,'memoryDecay',default_memoryDecay,validateNumeric)
addParameter(p,'uniformPriorThreshold',default_uniformPriorThreshold,validateNumeric)
addParameter(p,'surpriseThreshold',default_surpriseThreshold,validateNumeric)
addParameter(p,'anchorTolMerge',default_anchorTolMerge,validateNumeric)
addParameter(p,'anchorTolShift',default_anchorTolShift,validateNumeric)
addParameter(p,'anchorSamplingNoise',default_anchorSamplingNoise,validateNumeric)
addParameter(p,'spaceInterp',default_spaceInterp,validateNumeric)


addParameter(p,'errorThreshold',default_errorThreshold,validateNegative)

parse(p,agentType,varargin{:})

agentParams.relativeWidth         = p.Results.relativeWidth;  
agentParams.likelihoodSigma       = p.Results.likelihoodSigma;  
agentParams.likelihoodRange       = p.Results.likelihoodRange;  
agentParams.memoryDecay           = p.Results.memoryDecay;                
agentParams.uniformPriorThreshold = p.Results.uniformPriorThreshold;  
agentParams.surpriseThreshold     = p.Results.surpriseThreshold;
agentParams.resetWindow           = p.Results.resetWindow;
agentParams.resetFlag             = p.Results.resetFlag;             
agentParams.cacheSamplingMethod   = p.Results.cacheSamplingMethod;             
agentParams.cacheSize             = p.Results.cacheSize;                                      
agentParams.cacheFlag             = p.Results.cacheFlag;                                                                       
agentParams.anchorTolMerge        = p.Results.anchorTolMerge;                
agentParams.anchorTolShift        = p.Results.anchorTolShift;              
agentParams.anchorSamplingNoise   = p.Results.anchorSamplingNoise;                                                                           
agentParams.anchorInit            = p.Results.anchorInit;                    
agentParams.anchorTolScaling      = p.Results.anchorTolScaling;                                                                           
agentParams.anchorOrderMethod     = p.Results.anchorOrderMethod;                                                                    
agentParams.errorThreshold        = p.Results.errorThreshold;          
agentParams.timeInterp            = p.Results.timeInterp;                 
agentParams.spaceInterp           = p.Results.spaceInterp;                  
agentParams.nOptima               = p.Results.nOptima;                  


end



