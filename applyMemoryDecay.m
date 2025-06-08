function posterior = applyMemoryDecay(posterior,uniformPrior,belief)
% APPLYMEMORYDECAY Apply memory decay to a posterior belief.
%   posterior = APPLYMEMORYDECAY(posterior,uniformPrior,belief) applies a
%   memory decay to an input posterior belief by combining it with a prior
%   belief and renormalizing. By default, the prior is uniform, and so this
%   leads to a diffusion toward a flat prior over time.

posterior = normalizeBelief((1-belief.memoryDecay)*posterior + belief.memoryDecay.*uniformPrior);