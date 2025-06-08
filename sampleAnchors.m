function [anchors,peaks] = sampleAnchors(map,belief,sampler,planner,frac)
% SAMPLEANCHORS Sample anchor points from a 2D landscape.
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

% normalize the map
map = normalizeMap(map);

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

DKL = computeKLdiv(map,belief.uniformTargetPrior);
if DKL/belief.baseEntropy < sampler.uniformPriorThreshold

    % if the current posterior is sufficiently close to uniform, randomly
    % sample the maximum number of anchors
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
    anchors.thTol = sigma.*belief.size(1);
    anchors.rTol  = sigma.*belief.size(2);

    % add noise to anchors
    anchors.thCoords = anchors.thCoords + normrnd(0,sampler.samplingNoise*belief.size(1),[1,nk]);
    anchors.rCoords  = anchors.rCoords  + normrnd(0,sampler.samplingNoise*belief.size(2),[1,nk]);
    anchors = boundAnchors(anchors,belief);
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

function anchors = boundAnchors(anchors,belief)
% BOUNDANCHORS Returns a new set of anchors whose spatial extent does not 
% exceed a set of bounds.
%
%   anchors = BOUNDANCHORS(anchors,thBounds,rBounds) takes as input a set of
%   anchors and set of bounds defined in polar coordinates, and returns a 
%   new set of anchors that is constrained within those bounds.
%
%   See also: BOUNDTRAJECTORY

in = inpolygon(anchors.thCoords,anchors.rCoords,belief.thBoundary,belief.rBoundary);
anchorsToBound = find(~in);

for j = anchorsToBound
    dth = abs(anchors.thCoords(j)-belief.thBoundary');
    dr  = abs(anchors.rCoords(j) -belief.rBoundary' );

    [~,ind] = min(dr+dth);

    anchors.thCoords(j) = belief.thBoundary(ind);
    anchors.rCoords(j)  = belief.rBoundary(ind);
end

end
