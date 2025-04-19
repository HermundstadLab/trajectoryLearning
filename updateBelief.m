function [contextIDs,contextPosterior,posterior,cache,resetFlag,cacheFlag] = updateBelief(contextIDs,contextPrior,prior,likelihood,outcome,surprise,cache,belief)
% UPDATEBELIEF Updates Bayesian belief.
%   [posterior,resetFlag,entropy] = UPDATEBELIEF(contextPrior,prior,likelihood,outcome,surprise,cache,belief) 
%   uses the prior belief, together with the likelihood conditioned on the 
%   observed outcome, to update the posterior belief. If an input cache 
%   signal exceeds a threshold (specified in the 'belief' structure), the
%   belief is reset to a uniform prior. The function returns a binary flag
%   ('reset) about whether a cache occured, and returns the entropy of the
%   resulting posterior distribution.
%
%   See also: GETLIKELIHOOD


% determine whether to reset belief
if belief.resetFlag && numel(surprise)>belief.resetWindow-1 && all(surprise(end-belief.resetWindow+1:end)>belief.surpriseThreshold)

    % determine whether to cache
    if belief.cacheFlag && size(cache,3)<=belief.cacheSize
        % append current belief to cache 
        cache = cat(3,prior,cache);

        % update context IDs
        contextIDs = [[contextIDs(1,1),max(contextIDs(1,:))+1,contextIDs(1,2:end)];...
            [contextIDs(2,:),max(contextIDs(2,:))+1]];

        % update belief about context
        pOld = 1./(1+numel(contextPrior));
        pOld = pOld+.25;
        contextPosterior = [(1-pOld),pOld*contextPrior];

        cacheFlag = 1;
    else
        cacheFlag = 0;
    end
    
    % reset to uniform prior
    posterior = normalizeBelief(belief.mask.*ones(belief.np,belief.np));
    resetFlag     = 1;

else
    % else use prior and likelihood to update belief within current context
    posterior = normalizeBelief(prior.*likelihood(:,:,outcome)); 
    resetFlag = 0;
    cacheFlag = 0;

    % update belief about context
    for i=1:size(cache,3)
        probOutcome(i) = computeOutcomeProb(squeeze(cache(:,:,i)),likelihood,outcome);
    end
    contextPosterior = normalizeBelief(contextPrior.*probOutcome + .01*ones(size(contextPrior)));
end


end