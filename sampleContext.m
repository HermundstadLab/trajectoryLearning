function [prior,contextToRead] = sampleContext(contextPrior,cache,belief)
% SAMPLECONTEXT Sample anchor points from a probability distribution.
%
%   [prior,contextToRead] = SAMPLECONTEXT(contextPrior,cache,belief) uses
%   the prior over contexts ('contextPrior') to decide which context to 
%   read/sample, ('contextToRead'), which is then used to select the 
%   appropriate prior belief ('prior') from the  agent's cache.


if strcmp(belief.cacheSamplingMethod,'avg')
    prior = zeros(belief.np);
    for i=1:size(cache,3)
        prior = prior+contextPrior(i).*cache(:,:,i);
    end
    contextToRead = sum(contextPrior.*ones(1,belief.cacheSize));    % note that this need not be an integer, and is not
                                                                    % yet written to properly handle bimodal posteriors
else
    if strcmp(belief.cacheSamplingMethod,'MAP')
        [~,contextToRead] = max(contextPrior);
    elseif strcmp(belief.cacheSamplingMethod,'prop')
        c = rand();
        contextToRead = find(histcounts(c,[0,cumsum(contextPrior)])); 
    else
        error('unrecognized sampling method')
    end
    prior = squeeze(cache(:,:,contextToRead));
end
    
