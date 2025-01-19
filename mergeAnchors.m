function anchorsMerged = mergeAnchors(anchors,planner)
% MERGEANCHORS Determines an ordering for anchors that will be used to
% generate a curvilinear trajectory.
%
%   anchorsMerged = MERGEANCHORS(anchors,planner) takes as input a set of
%   ordered anchors (specified in polar coordinates and stored in the 
%   input structure 'anchors'), and merges anchors that are within a given
%   distance of one another (as specified by the 'planner' structure). This
%   is currently configured to trim one of the two anchors, but can be
%   modified to replace the two anchors with an intermediate anchor.
%
%   See also: ORDERANCHORS

% define new structure for merged anchors
anchorsMerged = anchors;

[~,allDists] = dpol(anchorsMerged.thCoords,anchorsMerged.rCoords); 
while any(allDists<planner.tol_merge)
    irem = find(allDists<planner.tol_merge,1,'first');

    % if the pair of anchors to be merged involve the first anchor, remove 
    % th second of the two anchors, rather than the first; otherwise,
    % remove the first of the two anchors:
    if irem==1
        irem = irem+1;
    end
    anchorsMerged.thCoords(irem) = [];
    anchorsMerged.rCoords( irem) = [];
    anchorsMerged.thTol(   irem) = [];
    anchorsMerged.rTol(    irem) = [];
    [~,allDists] = dpol(anchorsMerged.thCoords,anchorsMerged.rCoords); 
end
anchorsMerged.N = numel(anchorsMerged.thCoords);

if anchorsMerged.N<3
    error('merged all anchors; only home port remains')
end

end