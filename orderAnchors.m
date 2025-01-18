function anchorsOrdered = orderAnchors(anchors,planner)
% ORDERANCHORS Determines an ordering for anchors that will be used to
% generate a curvilinear trajectory.
%
%   anchorsOrdered = ORDERANCHORS(anchors,planner) takes as input a set of
%   unordered anchors (specified in polar coordinates and stored in the 
%   input structure 'anchors'), and returns a set of ordered anchors, 
%   ordered according the ordering specified by the 'planner' structure.
%
%   See also: PLANTRAJECTORY, OPTIMIZETRAJECTORY

nAnchors    = anchors.N;                                % number of anchors to order                      
nAnchorsMax = 5;                                        % number of anchors whose orderings
                                                        %    can be quickly enumerated

% order achors according to angle 
% (randomized ascending/descending sorting)
if rand()>0.5
    [anchors.thCoords,indsort] = sort(anchors.thCoords,'descend');
else
    [anchors.thCoords,indsort] = sort(anchors.thCoords,'ascend');
end
anchors.rCoords = anchors.rCoords(indsort);

if strcmp(planner.orderType,'angle')
    % return anchors ordered by angle, 
    % with home port appended

    anchorsOrdered.thCoords = [0,anchors.thCoords,0];
    anchorsOrdered.rCoords  = [0,anchors.rCoords, 0];
    anchorsOrdered.N        = numel(anchorsOrdered.thCoords);

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
        netDist    = zeros(1,size(permSet{i},1));           % stores summed distance btw anchors   

        % if final subset of anchors, append home port as anchor
        if i==nSets
            finalAnchorTH = 0;
            finalAnchorR  = 0;                              
        else
            finalAnchorTH = [];
            finalAnchorR  = [];
        end

        for j=1:size(permSet{i},1)
            % compute summed distance between anchors
            thTmp  = [baseSetTH,anchors.thCoords(permSet{i}(j,:)),finalAnchorTH];
            rTmp   = [baseSetR, anchors.rCoords( permSet{i}(j,:)),finalAnchorR ];
            [~,dr] = dpol(thTmp,rTmp);
            netDist = sum(dr);  
        end

        % find ordering that minimizes distance 
        [~,isel]  = min(netDist);
        baseSetTH = [baseSetTH,anchors.thCoords(permSet{i}(isel,:)),finalAnchorTH];
        baseSetR  = [baseSetR, anchors.rCoords( permSet{i}(isel,:)),finalAnchorR ];
    end

    %return ordered set of anchors, with home port appended
    anchorsOrdered.thCoords = baseSetTH;
    anchorsOrdered.rCoords  = baseSetR;
else
    error('unrecognized anchor ordering')
end


%  remove anchors within minimal distance of one another   
[~,allDists] = dpol(anchorsOrdered.thCoords,anchorsOrdered.rCoords); 
while any(allDists<planner.tol_merge)
    irem = find(allDists<planner.tol_merge,1,'first');
    anchorsOrdered.thCoords(irem) = [];
    anchorsOrdered.rCoords( irem) = [];
    [~,allDists] = dpol(anchorsOrdered.thCoords,anchorsOrdered.rCoords); 
end
anchorsOrdered.N = numel(anchorsOrdered.thCoords);
end
