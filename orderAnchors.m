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
anchors.thTol   = anchors.thTol(  indsort);
anchors.rTol    = anchors.rTol(   indsort);

if strcmp(planner.orderType,'angle')
    % return anchors ordered by angle, 
    % with home port appended

    anchorsOrdered.thCoords = [0,anchors.thCoords,0];
    anchorsOrdered.rCoords  = [0,anchors.rCoords, 0];
    anchorsOrdered.thTol    = [planner.thTol_shift,anchors.thTol,planner.thTol_shift];
    anchorsOrdered.rTol     = [planner.rTol_shift, anchors.rTol, planner.rTol_shift ];
    anchorsOrdered.N        = numel(anchorsOrdered.thCoords);

elseif strcmp(planner.orderType,'TSP')
    % solve approximate traveling salesman problem;
    % if more than nmax anchors, break into subproblems defined by
    %   permutation sets ('permset')

    nSets   = ceil(nAnchors/nAnchorsMax);
    permSet = cell(1,nSets);

    % initialize anchor coordinates and tolerances with home port anchor
    thCoords = 0;                                         
    rCoords  = 0;
    thTol = planner.thTol_shift;
    rTol  = planner.rTol_shift;
    for i=1:nSets
        permSet{i} = perms(((i-1)*nAnchorsMax+1):...        % subset of anchor orderings
            min(nAnchors,i*nAnchorsMax));               
        netDist    = zeros(1,size(permSet{i},1));           % stores summed distance btw anchors   

        % if final subset of anchors, append home port as anchor
        if i==nSets
            finalAnchor_thCoords = 0;
            finalAnchor_rCoords  = 0;   
            finalAnchor_thTol    = planner.thTol_shift;
            finalAnchor_rTol     = planner.rTol_shift;
        else
            finalAnchor_thCoords = [];
            finalAnchor_rCoords  = [];
            finalAnchor_thTol    = [];
            finalAnchor_rTol     = [];
        end

        for j=1:size(permSet{i},1)
            % compute summed distance between anchors
            thCoords_tmp = [thCoords,anchors.thCoords(permSet{i}(j,:)),finalAnchor_thCoords];
            rCoords_tmp  = [rCoords, anchors.rCoords( permSet{i}(j,:)),finalAnchor_rCoords ];
            [~,dr] = dpol(thCoords_tmp,rCoords_tmp);
            netDist = sum(dr);  
        end

        % find ordering that minimizes distance 
        [~,isel]  = min(netDist);
        thCoords = [thCoords,anchors.thCoords(permSet{i}(isel,:)),finalAnchor_thCoords];
        rCoords  = [rCoords, anchors.rCoords( permSet{i}(isel,:)),finalAnchor_rCoords ];
        thTol    = [thTol,   anchors.thTol(   permSet{i}(isel,:)),finalAnchor_thTol   ];
        rTol     = [rTol,    anchors.rTol(    permSet{i}(isel,:)),finalAnchor_rTol    ];
    end

    %return ordered set of anchors, with home port appended
    anchorsOrdered.thCoords = thCoords;
    anchorsOrdered.rCoords  = rCoords;
    anchorsOrdered.thTol    = thTol;
    anchorsOrdered.rTol     = rTol;
else
    error('unrecognized anchor ordering')
end


%  remove anchors within minimal distance of one another   
[~,allDists] = dpol(anchorsOrdered.thCoords,anchorsOrdered.rCoords); 
while any(allDists<planner.tol_merge)
    irem = find(allDists<planner.tol_merge,1,'first');
    anchorsOrdered.thCoords(irem) = [];
    anchorsOrdered.rCoords( irem) = [];
    anchorsOrdered.thTol(   irem) = [];
    anchorsOrdered.rTol(    irem) = [];
    [~,allDists] = dpol(anchorsOrdered.thCoords,anchorsOrdered.rCoords); 
end
anchorsOrdered.N = numel(anchorsOrdered.thCoords);
end
