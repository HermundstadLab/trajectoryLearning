function anchorsTrimmed = mergeAnchors(anchors,planner)
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
anchorsTrimmed = anchors;

[~,allDists] = dpol(anchorsTrimmed.thCoords,anchorsTrimmed.rCoords); 
while any(allDists<planner.tol_merge)
    irem = find(allDists<planner.tol_merge,1,'first');
    anchorsTrimmed.thCoords(irem) = [];
    anchorsTrimmed.rCoords( irem) = [];
    anchorsTrimmed.thTol(   irem) = [];
    anchorsTrimmed.rTol(    irem) = [];
    [~,allDists] = dpol(anchorsTrimmed.thCoords,anchorsTrimmed.rCoords); 
end
anchorsTrimmed.N = numel(anchorsTrimmed.thCoords);

end