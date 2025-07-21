function agent = generateAgent(agent,agentType,varargin)
% GENERATEAGENT Generate structures that define the artifical agent.
%
%   agent = generateAgent(agent,agentType,varargin) takes as input an
%   existing agent structure ('agent') that contains parameter values 
%   specified by the current environment, and a string that specifies the
%   type of agent to generate (currently configured only for a 'default' 
%   agents. Variable input arguments can be used to override default
%   parameter settings.
%
%   In order to construct the required 'agent' input structure, the 
%   function generateEnvironment must be run before running generateAgent.
%
%   REQUIRED INPUTS:
%   'agent'                         structure that specifies base parameters of
%                                   agent that are related to environment
%   'agentType'                     type of environment to generate ('default')
%
%   VARIABLE INPUTS:  
%   'likelihoodRange':              max range of likelihood (btw 0 and 1)  
%   'likelihoodSigma':              SD of gaussian likelihood, expressed as percentage of belief space (n.u.) 
%   'memoryDecay':                  rate of memory decay (measured as weighting to apply to uniform prior)
%   'uniformPriorThreshold':        threshold for detecting uniform prior (measured as the fraction DKL(prior||uniform)/DKL(delta||uniform) )
%   'surpriseThreshold':            threshold for caching posterior (outcome is 10x more surprising under current prior compared to uniform prior)
%   'resetWindow':                  number of successive timepoints for which surprise signal must exceed threshold
%   'resetFlag':                    determines whether to reset belief (options: true/false)
%   'cacheSamplingMethod':          method to use to sample context from cache ('avg','MAP','prop')
%   'cacheSize':                    maximum number of allowed items in cache (including working belief)  
%   'cacheFlag':                    determines whether agent can cache current belief (options: true/false)
%   'anchorTolMerge':               tolerance for merging anchors (n.u.)
%   'anchorTolShift':               tolerance for shifting anchors (n.u.)
%   'anchorSamplingNoise':          noise in sampling anchors (n.u.) 
%   'anchorInit':                   initial number of anchors
%   'anchorTolScaling':             determines whether to scale tolerancesaround individual anchors (options:true/false)
%   'anchorOrderMethod':            type of ordering to use for anchor points;  ('TSP','angle')
%   'errorThreshold':               error threshold for augmenting anchors points
%   'timeInterp':                   number of timepoints to use to interpolate trajectories
%   'spaceInterp':                  number of spatial points to interpolate trajectories around obstacles
%   'nOptima':                      number of optimzations to perform for initial heading
%
%   See also: GENERATEENVINMENT, GENERATETRIALSTRUCTURE

%--------------------------- parse inputs --------------------------------%

p = inputParser;

%------------ all beliefs ------------%
default_uniformPriorThreshold = 0.001;                  % threshold for detecting uniform prior (measured as the fraction DKL(prior||uniform)/DKL(delta||uniform) )

%----------- target belief -----------%
default_likelihoodRange       = 0.9;                    % max range of likelihood (btw 0 and 1)    
default_likelihoodSigma       = 0.075;                  % SD of gaussian likelihood, expressed as percentage of belief space (n.u.) 

default_resetFlag             = true;                   % determines whether to reset belief
                                                        % options: true/false
default_resetWindow           = 2;                      % number of successive timepoints for which surprise signal must exceed threshold

%---------- context belief -----------%
default_cacheFlag             = true;                   % determines whether agent can cache current belief
                                                        % options: true/false 
default_cacheSize             = 5;                      % maximum number of allowed items in cache (including working belief)                                                         
default_cacheSamplingMethod   = 'prop';                 % method to use to sample context from cache:
                                                        % options: 'MAP' (maximum a posteriori)
                                                        %          'prop' (proportional sampling)
                                                        %          'avg' (posterior average)                             
   
%------------- sampler ---------------%
default_anchorOrderMethod     = 'TSP';                  % type of ordering to use for anchor points; 
                                                        % options: 'TSP' (solves approx traveling salesman)
                                                        %          'angle' (sorts anchors based on angle)
default_anchorTolMerge        = 0.05;                   % tolerance for merging anchors (n.u.)
default_anchorTolShift        = 0.005;                  % tolerance for shifting anchors (n.u.)    
default_anchorInit            = 8;                      % initial number of anchors
default_anchorTolScaling      = true;                   % determines whether to scale tolerances around individual anchors 
                                                        % options: true (scale tolerances based on width of posterior peaks)
                                                        %          false (used fixed tolerance for all anchors       
%------------- error map -------------%
default_errorThreshold        = -0.05;                  % error threshold for augmenting anchors points                                                        

%--------- trajectory planner --------%                                                        
default_nOptima               = 5;                      % number of optimzations to perform for initial heading
default_timeInterp            = 100;                    % number of timepoints to use to interpolate trajectories
default_spaceInterp           = 0.25;                   % number of spatial points to interpolate trajectories around obstacles
                                                        %   (expressed as percentage of belief space per unit distance)

if strcmp(agentType,'default')

    default_memoryDecay           = 0;                  % rate of memory decay (measured as weighting to apply to uniform prior)
    default_surpriseThreshold     = 10;                 % threshold for caching posterior (outcome is 10x more surprising under current prior compared to uniform prior)
    default_anchorSamplingNoise   = 0;                  % noise in sampling anchors (n.u.) 
                                                        %   (should be ~less than the ratio of the target width to arena width)

elseif strcmp(agentType,'lossy') 

    default_memoryDecay           = 0.01;               % rate of memory decay (measured as weighting to apply to uniform prior)
    default_surpriseThreshold     = 5;                  % threshold for caching posterior (outcome is 5x more surprising under current prior compared to uniform prior)
    default_anchorSamplingNoise   = 0.01;               % noise in sampling anchors (n.u.) 
                                                        %   (should be ~less than the ratio of the target width to arena width)

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
validateStructure   = @(x) isstruct(x);

addRequired( p,'agent',validateStructure);
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

parse(p,agent,agentType,varargin{:})

%----------------------- build belief structure --------------------------%
agent.belief.rangeL         = p.Results.likelihoodRange;                   
agent.belief.sigmaL         = p.Results.likelihoodSigma*agent.belief.np;        % SD of gaussian likelihood (n.u.) 

agent.belief.npExclude      = p.Results.anchorTolMerge.*agent.belief.np;        % radial distance to exclude around home port (n.u.)
agent.belief.rMinAnchors    = agent.belief.rMin + ...                           % define minimum radius for anchors selected beyond home port
    p.Results.anchorTolMerge*(agent.belief.rMax-agent.belief.rMin);

agent.belief.memoryDecay         = p.Results.memoryDecay;                  
agent.belief.surpriseThreshold   = p.Results.surpriseThreshold;             
agent.belief.resetWindow         = p.Results.resetWindow;                   
agent.belief.resetFlag           = p.Results.resetFlag;                     

agent.belief.cacheSamplingMethod = p.Results.cacheSamplingMethod;           
agent.belief.cacheSize           = p.Results.cacheSize;                     
agent.belief.cacheFlag           = p.Results.cacheFlag;                     

agent.belief.mask                = createPosteriorMask(agent.belief);           % create posterior mask based on arena bounds
agent.belief.boundaryTol         = 1./agent.belief.np;                          % tolerance for determining boundary anchors
agent.belief.uniformTargetPrior  = normalizeBelief(...                          % uniform belief distribution
    agent.belief.mask.*ones(agent.belief.np,agent.belief.np));
agent.belief.baseEntropy         = computeEntropy(...                           % entropy of uniform belief distribution (used for scaling)
    agent.belief.uniformTargetPrior);  


%---------------------- build sampler structure --------------------------%

agent.sampler.minPeakDist           = round(p.Results.anchorTolMerge.*agent.belief.np);     % min distance between peaks in posterior (n.u.)
agent.sampler.minPeakHeight         = 1./(agent.belief.np.^2);                              % min height of peaks in posterior (n.u.)
agent.sampler.nAnchorsInit          = p.Results.anchorInit;                        
agent.sampler.errorThreshold        = p.Results.errorThreshold;                   
agent.sampler.samplingNoise         = p.Results.anchorSamplingNoise;              
agent.sampler.uniformPriorThreshold = p.Results.uniformPriorThreshold;             


%--------------------- build planner structure ---------------------------%

agent.planner.nInterp        = p.Results.timeInterp;                            % number of timepoints to use to interpolate trajectories
agent.planner.dt             = 1/agent.planner.nInterp;                         % time discretization
agent.planner.rScale         = agent.belief.size(2)./2;                         % used to scale the execution time of trajectory segments
                                                                                %   (defined in units of distance per time)  
agent.planner.tol_shift      = p.Results.anchorTolShift;                                                                                           
agent.planner.thTol_shift    = p.Results.anchorTolShift*agent.belief.size(1);   
agent.planner.rTol_shift     = p.Results.anchorTolShift*agent.belief.size(2);   
agent.planner.scaleTol       = p.Results.anchorTolScaling;                      
agent.planner.orderType      = p.Results.anchorOrderMethod;                     
agent.planner.nOptima        = p.Results.nOptima;       

agent.params = p.Results;

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