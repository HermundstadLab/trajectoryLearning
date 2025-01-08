function S = computeSurprise(P)
%COMPUTESURPRISE compute Shannon surprise of an event.
%   S = COMPUTESURPRISE(P) computes the surprise S of an event that occurs
%   with probability P.
%
%   See also: COMPUTEOUTCOMEPROB

S = -log2(P);
end