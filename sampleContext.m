function [prior,contextToSample] = sampleContext(contextPrior,contextIDs,cache,belief)
% SAMPLECONTEXT Sample anchor points from a probability distribution.
%
%   [prior,contextToSample] = SAMPLECONTEXT(contextPrior,cache,belief) uses
%   the prior over contexts ('contextPrior') to decide which context to 
%   sample, ('contextToSample'), which is then used to select the 
%   appropriate prior belief ('prior') from the  agent's cache.


if strcmp(belief.cacheSamplingMethod,'avg')
    prior = zeros(belief.np);
    for i=1:size(cache,3)
        prior = prior+contextPrior(i).*cache(:,:,i);
    end
    contextToSample = sum(contextPrior.*contextIDs);
else
    if strcmp(belief.cacheSamplingMethod,'MAP')
        [~,contextIndex] = max(contextPrior);
    elseif strcmp(belief.cacheSamplingMethod,'prop')
        c = rand();
        contextIndex = find(histcounts(c,[0,cumsum(contextPrior)])); 
    else
        error('unrecognized sampling method')
    end

    prior = squeeze(cache(:,:,contextIndex));
    contextToSample = contextIDs(contextIndex); 
end
    
