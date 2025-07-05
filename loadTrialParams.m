function trialParams = loadTrialParams(exptType)
% LOADTRIALPARAMS Loads parameters that specify the trial protocol for a 
% simulated experiment.
%
%   trialParams = loadTrialParams(exptType) takes as input a string that 
%   specifies the experimental protocal (currently configured for 
%   multi-target ('multiTarget') and single-target ('singleTarget')
%   environments without obstacles, and a single target environment with
%   a single obstacle ('obstacle'). It returns a structure that specifies
%   parameter settings for individual trials within a simulated experiment.  
%
%   See also: LOADENVIRONMENTPARAMS, LOADAGENTPARAMS

% the simulation is divided into blocks, each of which is defined by a
% target location, obstacle location (if any), and entry point:
%
% nBlocks:              number of blocks of trials 
% nTrialsPerBlocks:     number of trials per block
% targetArrangement:    specifies whether targets are placed in the center
%                           of the arena ('centered'), arranged radially
%                           ('radial'), or arranged randomly ('random')
% obstacleTrials:       binary vectory specifying presence/absence of 
%                           obstacle on each block of trials
% entryWall:            vector specifying whether the agent enters on the:
%                           0: south wall
%                           1: east wall
%                           2: north wall
%                           3: west wall

trialParams.exptType = exptType;
if strcmp(exptType,'multiTarget')
    trialParams.nBlocks             = 5;
    trialParams.nTrialsPerBlock     = 100;
    trialParams.targetArrangement   = 'radial';
    trialParams.obstacleTrials      = zeros(1,trialParams.nBlocks);
    trialParams.entryWall           = zeros(1,trialParams.nBlocks);

elseif strcmp(exptType,'singleTarget')
    trialParams.nBlocks             = 1;
    trialParams.nTrialsPerBlock     = 100;
    trialParams.targetArrangement   = 'radial';
    trialParams.obstacleTrials      = zeros(1,trialParams.nBlocks);
    trialParams.entryWall           = zeros(1,trialParams.nBlocks);

elseif strcmp(exptType,'obstacle')
    trialParams.nBlocks             = 1;
    trialParams.nTrialsPerBlock     = 100;
    trialParams.targetArrangement   = 'radial';
    trialParams.obstacleTrials      = ones(1,trialParams.nBlocks);
    trialParams.entryWall           = zeros(1,trialParams.nBlocks);

elseif strcmp(exptType,'interleaved obstacle')
    trialParams.nBlocks             = 3;
    trialParams.nTrialsPerBlock     = 100;
    trialParams.targetArrangement   = 'centered';
    trialParams.obstacleTrials      = zeros(1,trialParams.nBlocks);
    trialParams.obstacleTrials(2)   = 1;
    trialParams.entryWall           = zeros(1,trialParams.nBlocks);

elseif strcmp(exptType,'new entry')
    trialParams.nBlocks             = 3;
    trialParams.nTrialsPerBlock     = 20;
    trialParams.targetArrangement   = 'centered';
    trialParams.obstacleTrials      = zeros(1,trialParams.nBlocks);
    trialParams.entryWall           = zeros(1,trialParams.nBlocks);
    trialParams.entryWall(2:3)      = [1,3];

else
    error('unrecognized experiment type')
end
