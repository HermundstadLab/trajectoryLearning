function [belief,sampler,planner] = generateAgent(belief,planner,agentParams)
% GENERATEAGENT Generate structures that specify computational modules of
% the agent. 
%
%   [belief,sampler,planner] = GENERATEAGENT(belief,planner,agentParams) 
%   takes as input two structures that specify environment-specific 
%   properties of the agent's belief and planning modules (these will
%   be augmented within this function); it also takes as input a structure
%   that specifies parameter settings for the agent. The function returns 
%   structures that specify the augmented settings for the agent's belief,
%   sampler, and planner modules. 
%
%   In order to contruct the input 'belief' and 'planner' structures, the 
%   functions LOADENVIRONMENTPARAMS, LOADAGENTPARAMS, and 
%   GENERATEENVIRONMENT must be run before running LOADAGENTPARAMS.
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
belief.boundaryTol    = agentParams.anchorTolShift;                 % tolerance for determining boundary anchors


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