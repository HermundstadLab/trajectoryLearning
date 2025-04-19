function [anchors,peaks] = sampleAnchors(map,belief,sampler,planner,frac)
% SAMPLEANCHORS Sample anchor points from a probability distribution.
%
%   [anchors,peaks] = SAMPLEANCHORS(map,belief,sampler,planner,frac) finds 
%   peaks in a 2D landscape ('map') that are of a min height and separation 
%   (as specified in the 'belief' structure'). It then selects the minimum
%   subset of these peaks that cover a fraction 'frac' of their summed
%   values; this subset will be returned as sampled anchor points. If there
%   are no peaks in the map, this function randomly samples the maximum 
%   number of allowed peaks (as specified in the 'sampler' structure). The 
%   polar coordinates of these anchors are returned in the output structure 
%   'anchors'; the polar coordinates of all peaks are returned in the 
%   output structure 'peaks'.
%
%   See also: PEAKS2

% function defaults to sampling anchors that cover 50% of probability mass:
if nargin<5
    frac = 0.5;
end

% determine peaks in the map; these will be used to sample anchors              
mapTmp = map;                                 % define temporary variable for sampling
mapTmp(isnan(mapTmp)) = 0;                    % remove nans for sampling (allow borders to be peaks)
[pks,locs_r,locs_th] = peaks2(mapTmp,...      % determine peaks
    'MinPeakDistance',sampler.minPeakDist,...
    'MinPeakHeight',sampler.minPeakHeight);     
[pkssort,indsort] = sort(pks,'descend');

% extract coordinates of all anchors
peaks.thCoords = belief.thAxes(locs_th(indsort));                                          
peaks.rCoords  = belief.rAxes( locs_r(indsort)); 
peaks.values   = pkssort./sum(pkssort);

if numel(pks)<1
    % if there are no peaks in the map, randomly sample the maximum number of anchors
    nk = sampler.nAnchorsInit;

    % remove nans before sampling
    lininds = find(~isnan(map));

    % randomly sample nk anchors
    indperm = randperm(numel(lininds),nk);

    % extract polar coordinates of anchors
    [indr,indth] = ind2sub(size(map),lininds(indperm));
    anchors.thCoords = belief.thAxes(indth);                                           
    anchors.rCoords  = belief.rAxes( indr ); 
    anchors.thTol    = planner.thTol_shift*ones(1,numel(indth));
    anchors.rTol     = planner.rTol_shift*ones( 1,numel(indr ));
else
    
    % if there are peaks in the map, choose the smallest number of peaks
    % that cover half of the total peak probability mass
    nk = find(cumsum(peaks.values)>=frac,1,'first');

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

peaks.N = nk;

% remove any zero-amplitude anchors
indrem = find(anchors.rCoords==0);                                               
anchors.thCoords(indrem) = [];
anchors.rCoords( indrem) = [];
anchors.thTol(   indrem) = [];
anchors.rTol(    indrem) = [];

anchors.N = numel(anchors.rCoords);

end
