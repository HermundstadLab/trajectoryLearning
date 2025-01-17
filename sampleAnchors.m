function [thAnchors,rAnchors] = sampleAnchors(prior,belief,sampler)
% SAMPLEANCHORS Sample anchor points from a probability distribution.
%
%   [thAnchors,rAnchors] = SAMPLEANCHORS(prior,belief,sampler) finds
%   peaks in a prior probability distribution that are of a minimum height 
%   and separation (as specified in the 'belief' structure'). It then 
%   selects a fraction of these peaks as sampled anchor points; if there 
%   are no peaks in the distribution, this function randomly samples the 
%   maximum number of allowed peaks (as specified in the 'sampler' 
%   structure). 
%
%   See also: PEAKS2

% determine peaks in the prior; these will be used to sample anchors              
priorTmp = prior;                               % define temporary variable for sampling
priorTmp(isnan(priorTmp)) = 0;                  % remove nans for sampling (allow borders to be peaks)
[pks,locs_r,locs_th] = peaks2(priorTmp,...      % determine peaks
    'MinPeakDistance',sampler.minPeakDist,...
    'MinPeakHeight',sampler.minPeakHeight);     
[~,indsort] = sort(pks,'descend');

if numel(pks)<1
    % if there are no peaks in the prior, randomly sample the maximum number of anchors
    nk = sampler.nAnchorsMax;

    % remove nans before sampling
    lininds = find(~isnan(prior));

    % randomly sample nk anchors
    indperm = randperm(numel(lininds),nk);

    % extract polar coordinates of anchors
    [indr,indth] = ind2sub(size(prior),lininds(indperm));
    thAnchors = belief.thAxes(indth);                                           
    rAnchors  = belief.rAxes( indr);                                          

else
    % if there are peaks in the prior, choose fSample of the number of peaks to be anchors
    nk = max(1,floor(sampler.fAnchorsSample.*numel(pks)));

    % select top peaks to be anchor points
    indsel = indsort(1:nk);                                           
    indth  = locs_th(indsel);                                          
    indr   = locs_r( indsel);                                          

    % extract polar coordinates of anchors
    thAnchors = belief.thAxes(indth);                                          
    rAnchors  = belief.rAxes( indr);                                           
    
end

% remove any zero-amplitude anchors
indrem = find(rAnchors==0);                                               
rAnchors( indrem) = [];
thAnchors(indrem) = [];

end
