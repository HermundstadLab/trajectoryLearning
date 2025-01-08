function Pn = normalizeBelief(P)
% NORMALIZEBELIEF Normalize a belief distribution.
%   Pn = NORMALIZEBELIEF(P) normalizes a multi-dimensional probability 
%   distribution P to sum to one, excluding nans. 

Pn = P./sum(P(:),'omitnan');
end