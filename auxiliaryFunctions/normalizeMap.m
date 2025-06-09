function Pn = normalizeMap(P)
% NORMALIZEMAP Normalize an arbitrary distribution.
%   Pn = NORMALIZEMAP(P) normalizes a multi-dimensional distribution P 
%   that can, in principle, be negative. NORMALIZEMAP will return a
%   nonnegative distribution that sums to one, excluding nans. 

P(P<0)=eps;
Pn = P./sum(P(:),'omitnan');

end