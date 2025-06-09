function [targetPosterior,contextPosterior,contextToWrite,cache,resetFlag,cacheFlag] = updateBelief(targetPrior,targetLikelihood,outcome,contextPrior,contextToWrite,cache,surprise,belief)
% UPDATEBELIEF Updates Bayesian belief.
%   [posterior,contextPosterior,contextIDs,cache,resetFlag,cacheFlag] = 
%       UPDATEBELIEF(prior,likelihood,outcome,contextPrior,contextIDs,cache,surprise,belief) 
%   uses the prior belief about the target vector, together with the 
%   likelihood conditioned on the observed outcome, to update the posterior 
%   belief about the target vector. If an input surprise signal exceeds a 
%   threshold (specified in the 'belief' structure), the current state of
%   the posterior is stored in a cache as a new context, and the context
%   prior is updated to include the new context entry (with corresponding 
%   context IDs. The posterior is then reset to a uniform prior. The 
%   function returns two binary flag ('resetFlag' and 'cacheFlag') about
%   whether the posterior was reset and a cache occured.
%
%   See also: GETTARGETLIKELIHOOD


% determine whether to reset belief
if belief.resetFlag && numel(surprise)>belief.resetWindow-1 && all(surprise(end-belief.resetWindow+1:end)>belief.surpriseThreshold)
    % signal reset
    resetFlag = 1;

    % determine whether to cache
    if belief.cacheFlag
        % signal cache
        cacheFlag = 1;

        % stop writing to current cache entry, choose cache that is closest
        % to uniform to write
        for i=1:belief.cacheSize
            DKL(i) = computeKLdiv(cache(:,:,i),belief.uniformTargetPrior);
        end
        [~,contextToWrite] = min(DKL);
        contextsToUpdate   = find(contextPrior>0);

        % get new target posterior belief
        targetPosterior  = cache(:,:,contextToWrite);
        
        % update belief about context
        contextPosterior = zeros(size(contextPrior));
        contextPosterior( contextToWrite ) = 0.5;
        contextPosterior(contextsToUpdate) = 0.5*contextPrior(contextsToUpdate)./sum(contextPrior(contextsToUpdate));

    else
        % signal no cache
        cacheFlag = 0;

        % reset to uniform prior
        targetPosterior = normalizeBelief(belief.mask.*ones(belief.np,belief.np));

        % update cache
        cache(:,:,contextToWrite) = targetPosterior;
    end   

else
    % signal no reset
    resetFlag = 0;

    % signal no cache
    cacheFlag = 0;

    % use prior and likelihood to update belief within current context
    targetPosterior = normalizeBelief(targetPrior.*targetLikelihood(:,:,outcome)); 
    
    % update cache
    cache(:,:,contextToWrite) = targetPosterior;

    % update belief about context
    probOutcome = zeros(size(contextPrior));
    contextsToUpdate = find(contextPrior>0);
    for i=contextsToUpdate
        probOutcome(i) = computeOutcomeProb(squeeze(cache(:,:,i)),targetLikelihood,outcome);
    end
    contextPosterior = normalizeBelief(contextPrior.*probOutcome);

end


end