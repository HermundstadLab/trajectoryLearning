function [arena,belief,sampler,planner,trial,agentParams] = loadExperiment(exptType, envType, agentType)
% LOADEXPERIMENT Loads all data structures needed to run an simulated experiment.
%
%   [arena,belief,sampler,planner,trial] = LOADEXPERIMENT(exptType, envType, 
%   agentType) takes as input strings that specify the experiment type, 
%   environment type, and agent type, and returns structures that specify
%   properties of the arena and trial structure, as well as properties of 
%   the agent's belief, sampler, and planner modules. 
%
%   Inputs:
%       exptType:    'singleTarget', 'multiTarget', 'obstacle',
%       'interleaved obstacle', 'new entry'
%       envType:     'default'
%       agentType:   'default'
%   
%   See also: LOADAGENTPARAMS, LOADENVIRONMENTPARAMS, LOADTRIALPARAMS


if nargin<3
    envType   = 'default';
    agentType = 'default';
end


%-------------------- modify parameter settings --------------------------%

% default parameter values for specifying the environment, agent, and trial
% protocol are respectively stored in 'loadEnvironmentParams','loadAgentParams', 
% and 'loadTrialParams'.

%---------------- generate agent and environment -------------------------%
% NOTE: the following functions build upon one another, and must be called 
% in order

% current environment options: 'default'
% current agent options:       'default'

[arenaParams,...
    targetParams,...
    obstacleParams]  = loadEnvironmentParams(envType);
agentParams          = loadAgentParams(agentType);
[arena,...
    belief,...
    sampler,...
    planner,...
    target,...
    obstacle]        = generateAgentEnvironment(arenaParams,targetParams,obstacleParams,agentParams);

%------------------- generate trial protocol -----------------------------%
% current trial type options: 'singleTarget', 'multiTarget', 'obstacle'

trialParams = loadTrialParams(exptType);
trial = generateTrialStructure(arena,target,obstacle,planner,trialParams);
