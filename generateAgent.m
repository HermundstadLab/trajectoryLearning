function [belief,sampler,planner] = generateAgent(belief,agentParams)
% GENERATEAGENT Generate structures that specify computational modules of
% the agent. 
%
%   [belief,sampler,planner] = GENERATEAGENT(agentType,belief) takes as 
%   input a structure that specifies parameter settings for the agent, and  
%   another structure that specifies properties of the agent's belief that
%   are constrained by the environment (this structure will be augmented
%   within this function). The function returns structures that specify the
%   settings for the agent's belief, sampler, and planner modules. 
%
%   In order to contruct the input 'belief' structure, the functions 
%   loadEnvironmentParams and generateEnvironment must be run before 
%   running loadAgentParams.
%   
%   See also: GENERATEENVIRONMENT, GENERATETRIALSTRUCTURE


% NOTE:
% many of the following parameters are defined in normalized units (n.u.) 
% of the posterior discretization; others are defined in arbitrary units 
% of arena size (a.u.)

%----------------------- build belief structure --------------------------%

belief.rangeL = agentParams.likelihoodRange;                        % max range of likelihood (btw 0 and 1)    
belief.sigmaL = agentParams.likelihoodSigma*belief.np;              % SD of gaussian likelihood (n.u.) 

belief.npExclude      = agentParams.anchorTolMerge.*belief.np;      % radial distance to exclude around home port (n.u.)
belief.rMinAnchors    = belief.rMin + ...                           % define minimum radius for anchors selected beyond home port
    agentParams.anchorTolMerge*(belief.rMax-belief.rMin);
belief.cacheThreshold = agentParams.cacheThreshold;                 % surprise threshold for caching posterior;
belief.cacheWindow    = agentParams.cacheWindow;                    % number of successive timepoints that cache signal must exceed threshold
belief.mask           = createPosteriorMask(belief);                % create posterior mask based on arena bounds
belief.cache          = agentParams.cacheFlag;                      % determines whether to cache posterior


%---------------------- build sampler structure --------------------------%

sampler.minPeakDist    = agentParams.anchorTolMerge.*belief.np;     % min distance between peaks in posterior (n.u.)
sampler.minPeakHeight  = 1./(belief.np.^2);                         % min height of peaks in posterior (n.u.)
sampler.nAnchorsMax    = agentParams.anchorMax;                     % maximum number of anchors
sampler.errorThreshold = agentParams.errorThreshold;                % error threshold for augmenting anchors points


%--------------------- build planner structure ---------------------------%

planner.nInterp        = agentParams.timeInterp;                    % number of timepoints to use to interpolate trajectories
planner.rScale         = belief.size(2)./2;                         % used to scale the execution time of trajectory segments
                                                                    %   (defined in units of distance per time)

planner.tol_shift      = agentParams.anchorTolShift;                % default tolerance for shifting anchors                                                                    
planner.thTol_shift    = agentParams.anchorTolShift*belief.size(1); % default angular tolerance for shifting anchors (a.u.)
planner.rTol_shift     = agentParams.anchorTolShift*belief.size(2); % default radial tolerance for shifting anchors (a.u.)
planner.nxObstacle     = floor(agentParams.spaceInterp*belief.np);  % number of spatial points per unit length used to
                                                                    %   interpolate obstacle boundaries (sets boundary velocity)
planner.boundaryVelocity = 1./agentParams.spaceInterp;              % velocity at which agent moves along obstacle boundaries
                                                                    %   (this should be sufficiently fast; will o/w burn obstacle boundary into posterior)
planner.scaleTol       = agentParams.anchorTolScaling;              % determines whether to scale tolerances around individual anchors 
planner.orderType      = agentParams.anchorOrderMethod;             % type of ordering to use for anchor points; 


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