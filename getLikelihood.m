function L = getLikelihood(trajectory,belief)
% GETLIKELIHOOD Return 2D likelihood function in polar coordinates. 
%   L = GETLIKELIHOOD(trajectory,belief) converts an input trajectory from
%   cartesian to polar coordinates, convolves a gaussian function along
%   this polar trajectory, and normalizes the resulting likelihood function. 
%   The 'belief' input structure stores parameters of the convolved 
%   gaussian and the bounds of the normalization.
%
%   See also: UPDATEBELIEF

% convert trajectory to polar coordinates
[th,r] = cart2pol(trajectory.xCoords,trajectory.yCoords);

% define vectorizes inputs for Gaussian 
thAxes = repmat(belief.thNorm(:),size(th));
rAxes  = repmat(belief.rNorm(:), size(r));
thMean = repmat(th./belief.size(1),[belief.np.^2,1]);
rMean  = repmat(r./belief.size(2), [belief.np.^2,1]);
sigma  = belief.sigmaL./belief.np;

% compute Gaussian for each point along the trajectory
L = sum(gaussian(thAxes,rAxes,thMean,rMean,sigma,sigma),2,'omitnan');

% reshape, mask, and normalize likelihood     
L = reshape(L,[belief.np,belief.np]).*belief.mask;      
L = normalizeLikelihood(L,belief.rangeL);               
L = cat(3,L,(1-L));                 

end

function Ln = normalizeLikelihood(L,maxRange)
Ln = (L - min(L(:)))./(max(L(:)) - min(L(:)));
Ln = maxRange*Ln+(1-maxRange)./2;
end