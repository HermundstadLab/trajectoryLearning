function posterior = applyMemoryDecay(posterior,uniformPrior,normalize,belief)
% APPLYMEMORYDECAY Apply memory decay to a posterior belief.
%   posterior = APPLYMEMORYDECAY(posterior,uniformPrior,normalizeBelief,belief) 
%   applies a memory decay to an input posterior belief by combining it with 
%   a prior belief (and, if normalizeBelief = true, renormalizing). By default, 
%   the prior is uniform, and so this leads to a diffusion toward a flat prior 
%   over time.

posterior = (1-belief.memoryDecay)*posterior + belief.memoryDecay.*uniformPrior;
if normalize
    posterior = normalizeBelief(posterior);
end