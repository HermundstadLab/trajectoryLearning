function L = getLikelihood(trajectory,belief)
% GETLIKELIHOOD Return 2D likelihood function in polar coordinates. 
%   L = GETLIKELIHOOD(trajectory,belief) converts an input trajectory from
%   cartesian to polar coordinates, convolves a gaussian function along
%   this polar trajectory, and normalizes the resulting likelihood function. 
%   The 'belief' input structure stores parameters of the convolved 
%   gaussian and the bounds of the normalization.
%
%   See also: UPDATEBELIEF

L = zeros(size(belief.rNorm));
for i=1:numel(trajectory.xCoords)
    [theta,r] = cart2pol(trajectory.xCoords(i),trajectory.yCoords(i));
    lmap = gaussian(belief.thNorm,belief.rNorm,theta./belief.size(1),r./belief.size(2),belief.sigmaL./belief.np,belief.sigmaL./belief.np);
    lmap(isnan(lmap)) = 0;
    L = L+lmap;
end
L = L.*belief.mask;
L = normalizeLikelihood(L,belief.rangeL);
L = cat(3,L,(1-L));
end

function Ln = normalizeLikelihood(L,maxRange)
Ln = (L - min(L(:)))./(max(L(:)) - min(L(:)));
Ln = maxRange*Ln+(1-maxRange)./2;
end