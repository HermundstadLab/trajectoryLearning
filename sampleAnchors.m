function anchors = sampleAnchors(prior,belief,sampler,planner)
% SAMPLEANCHORS Sample anchor points from a probability distribution.
%
%   anchors = SAMPLEANCHORS(prior,belief,sampler,planner) finds peaks in a 
%   prior probability distribution that are of a min height and separation 
%   (as specified in the 'belief' structure'). It then selects a fraction 
%   of these peaks as sampled anchor points; if there are no peaks in the 
%   distribution, this function randomly samples the maximum number of 
%   allowed peaks (as specified in the 'sampler' structure). The polar 
%   coordinates of these anchors are stored in the output structure 
%   'anchors'.  
%
%   See also: PEAKS2

% determine peaks in the prior; these will be used to sample anchors              
priorTmp = prior;                               % define temporary variable for sampling
priorTmp(isnan(priorTmp)) = 0;                  % remove nans for sampling (allow borders to be peaks)
[pks,locs_r,locs_th] = peaks2(priorTmp,...      % determine peaks
    'MinPeakDistance',sampler.minPeakDist,...
    'MinPeakHeight',sampler.minPeakHeight);     
[pkssort,indsort] = sort(pks,'descend');

if numel(pks)<1
    % if there are no peaks in the prior, randomly sample the maximum number of anchors
    nk = sampler.nAnchorsMax;

    % remove nans before sampling
    lininds = find(~isnan(prior));

    % randomly sample nk anchors
    indperm = randperm(numel(lininds),nk);

    % extract polar coordinates of anchors
    [indr,indth] = ind2sub(size(prior),lininds(indperm));
    anchors.thCoords = belief.thAxes(indth);                                           
    anchors.rCoords  = belief.rAxes( indr ); 
    anchors.thTol    = planner.thTol_shift*ones(1,numel(indth));
    anchors.rTol     = planner.rTol_shift*ones( 1,numel(indr ));
else
    
    % if there are peaks in the prior, choose the smallest number of peaks
    % that cover half of the total peak probability mass
    nk = find(cumsum(pkssort./sum(pkssort))>.5,1,'first');

    % select top peaks to be anchor points
    indsel = indsort(1:nk);                                           
    indth  = locs_th(indsel);                                          
    indr   = locs_r( indsel);                                          

    % extract polar coordinates of anchors
    anchors.thCoords = belief.thAxes(indth);                                          
    anchors.rCoords  = belief.rAxes( indr); 
    
    % extract widths of peaks to use as tolerance for shifting anchors
    sigma = getPeakWidth(belief.np,pkssort(1:nk)',planner.tol_shift)./10;
    anchors.rTol  = sigma.*belief.size(2);
    anchors.thTol = sigma.*belief.size(1);
end

% remove any zero-amplitude anchors
indrem = find(anchors.rCoords==0);                                               
anchors.thCoords(indrem) = [];
anchors.rCoords( indrem) = [];
anchors.thTol(   indrem) = [];
anchors.rTol(    indrem) = [];

anchors.N = numel(anchors.rCoords);

end
