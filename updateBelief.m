function [posterior,cache,entropy] = updateBelief(prior,likelihood,outcome,cacheSignal,belief)
% UPDATEBELIEF Updates Bayesian belief.
%   [posterior,cache,entropy] = UPDATEBELIEF(prior,likelihood,outcome,cacheSignal,belief) 
%   uses the prior belief, together with the likelihood conditioned on the 
%   observed outcome, to update the posterior belief. If an input cache 
%   signal exceeds a threshold (specified in the 'belief' structure), the
%   belief is reset to a uniform prior. The function returns a binary flag
%   ('cache') about whether a cache occured, and returns the entropy of the
%   resulting posterior distribution.
%
%   See also: GETLIKELIHOOD


% determine whether to cache
if belief.cache && numel(cacheSignal)>belief.cacheWindow-1 && all(cacheSignal(end-belief.cacheWindow+1:end)>belief.cacheThreshold)
    % if cache signal drops below threshold, cache current posterior and revert to uniform posterior prior
    posterior = normalizeBelief(belief.mask.*ones(belief.np,belief.np));
    cache = 1;
else
    % else use prior and likelihood to update posterior
    posterior = normalizeBelief(prior.*likelihood(:,:,outcome)); 
    cache = 0;
end
entropy = computeEntropy(posterior);

end