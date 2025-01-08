function [belief,sampler,planner] = loadAgentParams(agentType,belief)
% LOADAGENTPARAMS Loads parameters that specify agent properties.
%
%   [belief,sampler,planner] = loadAgentParams(agentType,belief) takes as 
%   input a string that specifies the agent type (currently configured
%   only for a 'default' agent), and returns structures that specify the
%   settings for the agent's belief, sampler, and planner modules. 
%
%   This function requires a input 'belief' structure that will be
%   augmented by this function. In order to contruct this structure, the  
%   functions loadEnvironmentParams and generateEnvironment must be run 
%   before running loadAgentParams.
%   
%   See also: LOADENVIRONMENTPARAMS, GENERATEENVIRONMENT, LOADTRIALPARAMS

if strcmp(agentType,'default')

    % many of the following parameters are defined in normalized units (n.u.) of the posterior discretization
    % others are defined in arbitrary units of arena size (a.u.)

    belief.rangeL = 0.9;                        % max range of likelihood (btw 0 and 1)    
    belief.sigmaL = 0.075*belief.np;            % SD of gaussian likelihood (n.u.) 
    belief.npExclude = 0.05.*belief.np;         % radial distance to exclude around home port (n.u.)
    belief.cacheThreshold = 1.75;               % surprise threshold for caching posterior;
    belief.cacheWindow = 2;                     % number of successive timepoints that cache signal 
                                                % must exceed threshold
    belief.mask = createPosteriorMask(belief);  % create environment mask

    sampler.minPeakDist    = 0.05.*belief.np;   % min distance between peaks in posterior (n.u.)
    sampler.minPeakHeight  = 1./(belief.np.^2); % min height of peaks in posterior (n.u.)
    sampler.nAnchorsMax    = 10;                % maximum number of anchors
    sampler.fAnchorsSample = 0.5;               % fraction of anchors to sample
    sampler.errorThreshold = -0.1;              % error threshold for augmenting anchors points
    
    planner.nInterp     = 1000;                 % number of timepoints to use to interpolate trajectories
    planner.tScale      = belief.size(1)./2;    % used to scale the execution time of trajectory segments
    planner.tol_merge   = 0.1*belief.size(2);   % min radial distance for merging nearby anchors (a.u.)
    planner.tol_thShift = 0.005*belief.size(1); % max angular tolerance for shifting anchors
    planner.tol_rShift  = 0.005*belief.size(2); % max radial tolerance for shifting anchors
    planner.nxObstacle  = 4*belief.np;          % number of spatial points per unit length used
                                                %   to interpolate obstacle boundaries
    planner.orderType   = 'TSP';                % type of ordering to use for anchor points; 
                                                % options: 'TSP' (solves approx traveling salesman)
                                                %          'angle' (sorts anchors based on angle)

%elseif strcmp(agentType,'new agent type')      % uncomment to add new agent types
else
    error('unrecognized agent type')
end

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

