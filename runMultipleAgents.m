function multiAgentResults = runMultipleAgents(nAgents,arena,belief,sampler,planner,trial)
% RUNMULTIPLEAGENTS Simulates an agent that learns to intercept a target.
%
%   multiAgentResults = RUNMULTIPLEAGENTS(nAgents,arena,belief,sampler,planner,trial) 
%   runs multiple simulated agents in parallel, and stores the results in the output
%   structure 'multiAgentResults'.

% initialize arrays
[rewardRate,nAnchors,obstHits,boundary,...
    pReward,surprise,cache,entropy,...
    initAnchorLocs,finalAnchorLocs]        = deal([]);

nTrials = trial.nTrials;
parfor i=1:nAgents
    disp(['running agent ',num2str(i)]);
    singleAgentResults = runLearning(arena,belief,sampler,planner,trial);

    % store trajectory properties
    rewardRate = [rewardRate, singleAgentResults.trajectory.rewards         ];
    nAnchors   = [nAnchors,   singleAgentResults.trajectory.nAnchorsPlanned ];
    obstHits   = [obstHits,   singleAgentResults.trajectory.obstacleHits    ];
    boundary   = [boundary,   singleAgentResults.trajectory.boundaryFlag    ];

    % store belief properties
    pReward    = [pReward,    singleAgentResults.belief.probReward          ];
    surprise   = [surprise,   singleAgentResults.belief.cacheSignal         ];
    cache      = [cache,      singleAgentResults.belief.cache               ];
    entropy    = [entropy,    singleAgentResults.belief.entropy             ];

    % extract and store anchor properties
    [xx0,yy0] = pol2cart(singleAgentResults.trajectory.executed{1  }.anchors.thCoords,singleAgentResults.trajectory.executed{1  }.anchors.rCoords);
    [xxF,yyF] = pol2cart(singleAgentResults.trajectory.executed{100}.anchors.thCoords,singleAgentResults.trajectory.executed{100}.anchors.rCoords);
    n0 = singleAgentResults.trajectory.executed{1}.anchors.N;
    nF = singleAgentResults.trajectory.executed{100}.anchors.N;

    initAnchorLocs = [initAnchorLocs,    [xx0(2:end-1);yy0(2:end-1);repmat(n0-2,[1,n0-2]);repmat(i,[1,n0-2]) ] ];
    if ~singleAgentResults.trajectory.executed{100}.boundaryFlag
        finalAnchorLocs = [finalAnchorLocs,    [xxF(2:end-1);yyF(2:end-1);repmat(nF-2,[1,nF-2]);repmat(i,[1,nF-2]);...
            repmat(mean(singleAgentResults.trajectory.rewards(ceil(nTrials/2)+1:nTrials)),[1,nF-2])] ];
    end
end

multiAgentResults.trialProtocol = trial;

% store trajectory properties
multiAgentResults.trajectory.rewardRate   = rewardRate;
multiAgentResults.trajectory.nAnchors     = nAnchors;
multiAgentResults.trajectory.obstacleHits = obstHits;
multiAgentResults.trajectory.boundaryRuns = boundary;

% store anchor properties
multiAgentResults.trajectory.initialAnchors.xCoords    = initAnchorLocs(1,:);
multiAgentResults.trajectory.initialAnchors.yCoords    = initAnchorLocs(2,:);
multiAgentResults.trajectory.initialAnchors.N          = initAnchorLocs(3,:);
multiAgentResults.trajectory.initialAnchors.agentIndex = initAnchorLocs(4,:);

multiAgentResults.trajectory.finalAnchors.xCoords    = finalAnchorLocs(1,:);
multiAgentResults.trajectory.finalAnchors.yCoords    = finalAnchorLocs(2,:);
multiAgentResults.trajectory.finalAnchors.N          = finalAnchorLocs(3,:);
multiAgentResults.trajectory.finalAnchors.agentIndex = finalAnchorLocs(4,:);
multiAgentResults.trajectory.finalAnchors.avgReward  = finalAnchorLocs(5,:);

% store belief properties
multiAgentResults.belief.pReward  = pReward;
multiAgentResults.belief.surprise = surprise;
multiAgentResults.belief.cache    = cache;
multiAgentResults.belief.entropy  = entropy;