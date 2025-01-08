function Poutcome = computeOutcomeProb(prior,likelihood,outcome)
% COMPUTEOUTCOMEPROB Computes the probability of an observed outcome under
% a given prior belief.
%   Poutcome = COMPUTEOUTCOMEPROB(prior,likelihood,outcome,belief) uses the
%   current prior and the executed trajectory (stored in the likelihood) to
%   compute the probability of the observed outcome. 
%   
%   See also: COMPUTESURPRISE

Poutcome = sum(sum(prior.*likelihood(:,:,outcome),'omitnan'),'omitnan');

end