function D = computeKLdiv(P,Q)
%COMPUTEKLDIV compute KL divergence of probability distribution P from Q.
%
%   See also: COMPUTEOUTCOMEPROB, COMPUTESURPRISE

D = sum(P(:).*log2(P(:)./Q(:)),'omitnan');
end