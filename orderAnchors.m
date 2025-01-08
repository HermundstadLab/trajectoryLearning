function [thAnchorsOrdered,rAnchorsOrdered] = orderAnchors(thAnchors,rAnchors,planner)
% ORDERANCHORS Determines an ordering for anchors that will be used to
% generate a curvilinear trajectory.
%
%   [thAnchorsOrdered,rAnchorsOrdered] = ORDERANCHORS(thAnchors,rAnchors,...
%   planner) takes as input a set of unordered anchors (specified in polar
%   coordinates), and returns a set of ordered anchors according the
%   ordering specified by the 'planner' structure.
%
%   See also: PLANTRAJECTORY, OPTIMIZETRAJECTORY

nAnchors = numel(rAnchors);                             % number of anchors to order                      
nAnchorsMax = 5;                                        % number of anchors whose orderings
                                                        %    can be quickly enumerated

% order achors according to angle 
% (randomized ascending/descending sorting)
if rand()>0.5
    [thAnchors,indsort] = sort(thAnchors,'descend');
else
    [thAnchors,indsort] = sort(thAnchors,'ascend');
end
rAnchors = rAnchors(indsort);

if strcmp(planner.orderType,'angle')
    % return anchors ordered by angle, 
    % with home port appended

    thAnchorsOrdered = [0,thAnchors,0];
    rAnchorsOrdered  = [0,rAnchors, 0];

elseif strcmp(planner.orderType,'TSP')
    % solve approximate traveling salesman problem;
    % if more than nmax anchors, break into subproblems

    nSets   = ceil(nAnchors/nAnchorsMax);
    permSet = cell(1,nSets);

    baseSetR   = 0;                                         % previous ordering to append
    baseSetTH  = 0;
    for i=1:nSets
        permSet{i} = perms(((i-1)*nAnchorsMax+1):...        % subset of anchor orderings
            min(nAnchors,i*nAnchorsMax));               
        netDist       = zeros(1,size(permSet{i},1));        % stores summed distance btw anchors   

        % if final subset of anchors, append home port as anchor
        if i==nSets
            finalAnchorTH = 0;
            finalAnchorR  = 0;                              
        else
            finalAnchorTH = [];
            finalAnchorR  = [];
        end

        for j=1:size(permSet{i},1)
            %compute summed distance between anchors
            thTmp  = [baseSetTH,thAnchors(permSet{i}(j,:)),finalAnchorTH];
            rTmp   = [baseSetR, rAnchors( permSet{i}(j,:)),finalAnchorR ];
            [~,dr] = dpol(thTmp,rTmp);
            netDist = sum(dr);  
        end

        % find ordering that minimizes distance 
        [~,isel]  = min(netDist);
        baseSetTH = [baseSetTH,thAnchors(permSet{i}(isel,:)),finalAnchorTH];
        baseSetR  = [baseSetR, rAnchors( permSet{i}(isel,:)),finalAnchorR ];
    end

    %return ordered set of anchors, with home port appended
    thAnchorsOrdered = baseSetTH;
    rAnchorsOrdered  = baseSetR;
else
    error('unrecognized anchor ordering')
end


%  remove anchors within minimal distance of one another   
[~,allDists] = dpol(thAnchorsOrdered,rAnchorsOrdered); 
while any(allDists<planner.tol_merge)
    irem = find(allDists<planner.tol_merge,1,'first');
    thAnchorsOrdered(irem) = [];
    rAnchorsOrdered( irem) = [];
    [~,allDists] = dpol(thAnchorsOrdered,rAnchorsOrdered); 
end

end
