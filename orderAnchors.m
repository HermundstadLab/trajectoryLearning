function [anchorsOrdered,anchorIDsOrdered] = orderAnchors(anchors,planner,trial,trialID)
% ORDERANCHORS Determines an ordering for anchors that will be used to
% generate a curvilinear trajectory.
%
%   anchorsOrdered = ORDERANCHORS(anchors,planner) takes as input a set of
%   unordered anchors (specified in polar coordinates and stored in the 
%   input structure 'anchors'), and returns a set of ordered anchors, 
%   ordered according the ordering specified by the 'planner' structure.
%
%   See also: MERGEANCHORS, PLANTRAJECTORY, OPTIMIZETRAJECTORY

nAnchors    = anchors.N;                                % number of anchors to order        
anchorIDs   = 1:nAnchors;                               % anchor IDs (to keep track of reorderings)
nAnchorsMax = 5;                                        % number of anchors whose orderings
                                                        %    can be quickly enumerated
                                                   

% compute vector from entry point to home port
xEntry = trial.arena.agent.entryPoint(trial.blockIDs(trialID),1);
yEntry = trial.arena.agent.entryPoint(trial.blockIDs(trialID),2);
[thEntry,rEntry] = cart2pol(xEntry,yEntry);

% order achors according to angle 
% (randomized ascending/descending sorting)
if rand()>0.5
    [~,indsort] = sort(anchors.thCoords-thEntry,'descend');
else
    [~,indsort] = sort(anchors.thCoords-thEntry,'ascend');
end
anchors.thCoords = anchors.thCoords(indsort);
anchors.rCoords  = anchors.rCoords( indsort);
anchors.thTol    = anchors.thTol(   indsort);
anchors.rTol     = anchors.rTol(    indsort);
anchorIDs        = anchorIDs(       indsort);

if strcmp(planner.orderType,'angle')
    % return anchors ordered by angle, 
    % with home port appended

    anchorsOrdered.thCoords = [thEntry,anchors.thCoords,0];
    anchorsOrdered.rCoords  = [rEntry, anchors.rCoords, 0];
    anchorsOrdered.thTol    = [planner.thTol_shift,anchors.thTol,planner.thTol_shift];
    anchorsOrdered.rTol     = [planner.rTol_shift, anchors.rTol, planner.rTol_shift ];
    anchorsOrdered.N        = numel(anchorsOrdered.thCoords);

    anchorIDsOrdered        = anchorIDs;

elseif strcmp(planner.orderType,'TSP')
    % solve approximate traveling salesman problem;
    % if more than nmax anchors, break into subproblems defined by
    %   permutation sets ('permset')

    nSets   = ceil(nAnchors/nAnchorsMax);
    permSet = cell(1,nSets);
    anchorIDsOrdered = [];

    % initialize anchor coordinates and tolerances with entry point
    thCoords = thEntry;                                         
    rCoords  = rEntry;
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
        [~,isel] = min(netDist);
        thCoords = [thCoords,anchors.thCoords(permSet{i}(isel,:)),finalAnchor_thCoords];
        rCoords  = [rCoords, anchors.rCoords( permSet{i}(isel,:)),finalAnchor_rCoords ];
        thTol    = [thTol,   anchors.thTol(   permSet{i}(isel,:)),finalAnchor_thTol   ];
        rTol     = [rTol,    anchors.rTol(    permSet{i}(isel,:)),finalAnchor_rTol    ];

        anchorIDsOrdered = [anchorIDsOrdered,anchorIDs(permSet{i}(isel,:))];
    end

    %return ordered set of anchors, with home port appended
    anchorsOrdered.thCoords = thCoords;
    anchorsOrdered.rCoords  = rCoords;
    anchorsOrdered.thTol    = thTol;
    anchorsOrdered.rTol     = rTol;

    anchorIDsOrdered = [nan,anchorIDsOrdered,nan];
else
    error('unrecognized anchor ordering')
end
% store number of anchor points
anchorsOrdered.N = numel(anchorsOrdered.thCoords);

end
