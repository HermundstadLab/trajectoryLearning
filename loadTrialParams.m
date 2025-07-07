function trialParams = loadTrialParams(exptType,varargin)
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

p = inputParser;
if strcmp(exptType,'multiTarget')
    default_nBlocks             = 5;
    default_nTrialsPerBlock     = 100;
    default_targetArrangement   = 'radial';
    default_obstacleTrials      = zeros(1,default_nBlocks);
    default_entryWall           = zeros(1,default_nBlocks);

elseif strcmp(exptType,'singleTarget')
    default_nBlocks             = 1;
    default_nTrialsPerBlock     = 100;
    default_targetArrangement   = 'radial';
    default_obstacleTrials      = zeros(1,default_nBlocks);
    default_entryWall           = zeros(1,default_nBlocks);

elseif strcmp(exptType,'obstacle')
    default_nBlocks             = 1;
    default_nTrialsPerBlock     = 100;
    default_targetArrangement   = 'radial';
    default_obstacleTrials      = ones(1,default_nBlocks);
    default_entryWall           = zeros(1,default_nBlocks);

elseif strcmp(exptType,'interleaved obstacle')
    default_nBlocks             = 3;
    default_nTrialsPerBlock     = 100;
    default_targetArrangement   = 'centered';
    default_obstacleTrials      = zeros(1,default_nBlocks);
    default_obstacleTrials(2)   = 1;
    default_entryWall           = zeros(1,default_nBlocks);

elseif strcmp(exptType,'new entry')
    default_nBlocks             = 4;
    default_nTrialsPerBlock     = 20;
    default_targetArrangement   = 'centered';
    default_obstacleTrials      = zeros(1,default_nBlocks);
    default_entryWall           = zeros(1,default_nBlocks);
    default_entryWall(2:4)      = [1,2,3];

else
    error('unrecognized experiment type')
end

validExptTypes = {'new entry','interleaved obstacle','osbtacle','singleTarget','multiTarget'};
checkExptTypes = @(x) any(validatestring(x,validExptTypes));

validTargetArrangements = {'centered','radial','random'};
checkTargetArrangements = @(x) any(validatestring(x,validTargetArrangements));

addRequired( p,'exptType',checkExptTypes);
addOptional( p,'targetArrangement',default_targetArrangement,checkTargetArrangements)
addParameter(p,'nBlocks',default_nBlocks,@isnumeric)
addParameter(p,'nTrialsPerBlock',default_nTrialsPerBlock,@isnumeric)
addParameter(p,'obstacleTrials',default_obstacleTrials,@isnumeric)
addParameter(p,'entryWall',default_entryWall,@isnumeric)

parse(p,exptType,varargin{:})
trialParams.exptType            = p.Results.exptType;
trialParams.nBlocks             = p.Results.nBlocks;
trialParams.nTrialsPerBlock     = p.Results.nTrialsPerBlock;
trialParams.targetArrangement   = p.Results.targetArrangement;
trialParams.obstacleTrials      = p.Results.obstacleTrials;
trialParams.entryWall           = p.Results.entryWall;

% adjust vector lengths if necessary
if numel(trialParams.obstacleTrials)~=trialParams.nBlocks
    if ~strcmp(trialParams.exptType,'obstacle')
        disp('defaulting to no obstacles')
        trialParams.obstacleTrials = zeros(1,trialParams.nBlocks);
    else
        disp('defaulting to obstacles on every block')
        trialParams.obstacleTrials = ones(1,trialParams.nBlocks);
    end
end

if numel(trialParams.entryWall)~=trialParams.nBlocks
    if ~strcmp(trialParams.exptType,'new entry')
        disp('defaulting arena entries to origin')
        trialParams.entryWall = zeros(1,trialParams.nBlocks);
    else
        disp('defaulting to random selection of entrance wall')
        trialParams.entryWall = zeros(1,trialParams.nBlocks);
        for i=1:trialParams.nBlocks
            trialParams.entryWall(i) = randperm(trialParams.nBlocks,1)-1;
        end
    end
end