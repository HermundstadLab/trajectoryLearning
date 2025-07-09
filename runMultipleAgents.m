function multiAgentResults = runMultipleAgents(nAgents,agent,trial)
% RUNMULTIPLEAGENTS Simulates an agent that learns to intercept a target.
%
%   multiAgentResults = RUNMULTIPLEAGENTS(nAgents,agent,trial) runs
%   multiple simulated agents in parallel, and stores the results in the 
%   output structure 'multiAgentResults'.


%---------------------- extract agent structures -------------------------%
belief  = agent.belief;

%-------------------------  initialize arrays ----------------------------%
[rewardRate,nAnchors,obstHits,boundary,...
    rewardProb,outcomeSurprise,resetFlag,cacheFlag,...
    targetPosteriorEntropy,contextPosteriorEntropy,...
    initAnchorLocs,finalAnchorLocs,distance,latency,...
    contextPosterior,sampledContext,estimatedContext  ] = deal([]);

nTrials = trial.nTrials;

for i=1:nAgents
    disp(['running agent ',num2str(i)]);
    singleAgentResults = runSingleAgent(agent,trial);

    contextPosteriorTemp = nan(belief.cacheSize,nTrials);
    np = size(singleAgentResults.belief.context.posteriors,2);
    contextPosteriorTemp(:,1:np) = singleAgentResults.belief.context.posteriors;

    % store trajectory properties
    rewardRate = [rewardRate, singleAgentResults.trajectory.rewards            ];
    nAnchors   = [nAnchors,   singleAgentResults.trajectory.augmented.nAnchors ];
    obstHits   = [obstHits,   singleAgentResults.trajectory.obstacleHits       ];
    boundary   = [boundary,   singleAgentResults.trajectory.boundaryFlag       ];
    distance   = [distance,   singleAgentResults.trajectory.executed.distance  ];
    latency    = [latency,    singleAgentResults.trajectory.executed.latency   ];

    % store belief properties
    rewardProb      = [rewardProb,      singleAgentResults.belief.target.rewardProb      ];
    outcomeSurprise = [outcomeSurprise, singleAgentResults.belief.target.outcomeSurprise ];
    resetFlag       = [resetFlag,       singleAgentResults.belief.target.resetFlag       ];
    cacheFlag       = [cacheFlag,       singleAgentResults.belief.target.cacheFlag       ];

    targetPosteriorEntropy  = [targetPosteriorEntropy, singleAgentResults.belief.target.posteriorEntropy ];
    contextPosteriorEntropy = [contextPosteriorEntropy,singleAgentResults.belief.context.posteriorEntropy];

    contextPosterior = [contextPosterior, contextPosteriorTemp                        ];
    estimatedContext = [estimatedContext, singleAgentResults.belief.context.toWrite   ];
    sampledContext   = [sampledContext,   singleAgentResults.belief.context.toRead    ];

    % extract and store anchor properties
    [xx0,yy0] = pol2cart(singleAgentResults.trajectory.executed.path{   1   }.anchors.thCoords,singleAgentResults.trajectory.executed.path{   1   }.anchors.rCoords);
    [xxF,yyF] = pol2cart(singleAgentResults.trajectory.executed.path{nTrials}.anchors.thCoords,singleAgentResults.trajectory.executed.path{nTrials}.anchors.rCoords);
    n0 = singleAgentResults.trajectory.executed.path{   1   }.anchors.N;
    nF = singleAgentResults.trajectory.executed.path{nTrials}.anchors.N;

    initAnchorLocs = [initAnchorLocs,    [xx0(2:end-1);yy0(2:end-1);repmat(n0-2,[1,n0-2]);repmat(i,[1,n0-2]) ] ];
    if ~singleAgentResults.trajectory.executed.path{nTrials}.boundaryFlag
        finalAnchorLocs = [finalAnchorLocs,    [xxF(2:end-1);yyF(2:end-1);repmat(nF-2,[1,nF-2]);repmat(i,[1,nF-2]);...
            repmat(mean(singleAgentResults.trajectory.rewards(ceil(nTrials/2)+1:nTrials)),[1,nF-2])] ];
    end
end

multiAgentResults.nAgents = nAgents;

multiAgentResults.trialProtocol = trial;

% store trajectory properties
multiAgentResults.trajectory.rewardRate   = rewardRate;
multiAgentResults.trajectory.nAnchors     = nAnchors;
multiAgentResults.trajectory.distance     = distance;
multiAgentResults.trajectory.latency      = latency;
multiAgentResults.trajectory.obstacleHits = obstHits;
multiAgentResults.trajectory.boundaryRuns = boundary;

% store anchor properties
multiAgentResults.trajectory.initialAnchors.xCoords    = initAnchorLocs(1,:);
multiAgentResults.trajectory.initialAnchors.yCoords    = initAnchorLocs(2,:);
multiAgentResults.trajectory.initialAnchors.N          = initAnchorLocs(3,:);
multiAgentResults.trajectory.initialAnchors.agentIndex = initAnchorLocs(4,:);

multiAgentResults.trajectory.finalAnchors.xCoords      = finalAnchorLocs(1,:);
multiAgentResults.trajectory.finalAnchors.yCoords      = finalAnchorLocs(2,:);
multiAgentResults.trajectory.finalAnchors.N            = finalAnchorLocs(3,:);
multiAgentResults.trajectory.finalAnchors.agentIndex   = finalAnchorLocs(4,:);
multiAgentResults.trajectory.finalAnchors.avgReward    = finalAnchorLocs(5,:);

% store belief properties
multiAgentResults.belief.target.rewardProb        = rewardProb;
multiAgentResults.belief.target.outcomeSurprise   = outcomeSurprise;
multiAgentResults.belief.target.resetFlag         = resetFlag;
multiAgentResults.belief.target.cacheFlag         = cacheFlag;
multiAgentResults.belief.target.posteriorEntropy  = targetPosteriorEntropy;

multiAgentResults.belief.context.posteriors       = reshape(contextPosterior,[nTrials,belief.cacheSize,nAgents]);
multiAgentResults.belief.context.estimated        = estimatedContext;
multiAgentResults.belief.context.sampled          = sampledContext;
multiAgentResults.belief.context.posteriorEntropy = contextPosteriorEntropy;

% store entropy of flat prior, for plotting
multiAgentResults.belief.target.posteriorEntropyFlat = belief.baseEntropy;