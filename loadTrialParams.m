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

trialParams.exptType = exptType;
if strcmp(exptType,'multiTarget')
    trialParams.nBlocks             = 2;
    trialParams.nTargets            = trialParams.nBlocks;
    trialParams.nTrialsPerTarget    = 100;
    trialParams.targetArrangement   = 'radial';
    trialParams.obstacle            = 'false';
    trialParams.nObstacles          = 0;
    trialParams.obstacleArrangement = 'none';

elseif strcmp(exptType,'singleTarget')
    trialParams.nBlocks             = 1;
    trialParams.nTargets            = trialParams.nBlocks;
    trialParams.nTrialsPerTarget    = 100;
    trialParams.targetArrangement   = 'radial';
    trialParams.obstacle            = 'false';
    trialParams.nObstacles          = 0;
    trialParams.obstacleArrangement = 'none';

elseif strcmp(exptType,'obstacle')
    trialParams.nBlocks             = 1;
    trialParams.nTargets            = trialParams.nBlocks;
    trialParams.nTrialsPerTarget    = 100;
    trialParams.targetArrangement   = 'radial';
    trialParams.obstacle            = 'true';
    trialParams.nObstacles          = trialParams.nBlocks;
    trialParams.obstacleArrangement = 'centered';

else
    error('unrecognized experiment type')
end
