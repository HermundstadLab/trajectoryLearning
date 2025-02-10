function H = computeEntropy(P)
%COMPUTEENTROPY compute Shannon entropy of a probability distribution event.
%   H = COMPUTEENTROPY(P) computes the entropy (in bits) of a normalized
%   probability distribution P.
%
%   See also: COMPUTESURPRISE

H = -sum(P(:).*log2(P(:)), 'omitnan');