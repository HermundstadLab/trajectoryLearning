function [prior,contextEstimate] = sampleContext(contextPrior,contextIDs,cache,belief)
% SAMPLECONTEXT Sample anchor points from a probability distribution.
%
%   [prior,contextEstimate] = SAMPLECONTEXT(contextPrior,cache,belief) uses
%   the prior over contexts ('contextPrior') to estimate the current context,
%   ('contextEstimate'), which is then used to select the appropriate prior
%   belief ('prior') from the  agent's cache.


if strcmp(belief.cacheSamplingMethod,'MAP')
    [~,contextIndex] = max(contextPrior);
elseif strcmp(belief.cacheSamplingMethod,'prop')
    c = rand();
    contextIndex = find(histcounts(c,[0,cumsum(contextPrior)])); 
else
    error('unrecognized sampling method')
end
    
prior = squeeze(cache(:,:,contextIndex));
contextEstimate = contextIDs(:,contextIndex); 