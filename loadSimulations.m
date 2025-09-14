function [agent,environment,trial,results,dateStr] = loadSimulations(simType,exptType,reset,cache,learningSpeed)

if nargin<5
    learningSpeed = [];
end

if strcmp(exptType,'multiTarget')
    nBlocks    = 5;
    nObstacles = 0;
    entry      = 'default';

elseif strcmp(exptType,'singleTarget')
    nBlocks    = 1;
    nObstacles = 0;
    entry      = 'default';

elseif strcmp(exptType,'targetSwitch')
    nBlocks    = 2;
    nObstacles = 0;
    entry      = 'default';

elseif strcmp(exptType,'obstacle')
    nBlocks    = 1;
    nObstacles = 1;
    entry      = 'default';

elseif strcmp(exptType,'interleavedObstacle')
    nBlocks    = 3;
    nObstacles = 1;
    entry      = 'default';

elseif strcmp(exptType,'newEntry')
    nBlocks    = 4;
    nObstacles = 0;
    entry      = 'new';

else
    error('unrecognized experiment type')
end


if strcmp(simType,'single')
    simStr = 'singleAgent_';
    speedStr = ['_',num2str(learningSpeed),'runs'];
elseif strcmp(simType,'multiple')
    simStr = 'multiAgent_';
else
    error('unrecognized simulation type')
end

if nBlocks>1
    blockStr = [num2str(nBlocks),'Blocks_'];
else
    blockStr = [num2str(nBlocks),'Block_'];
end

if nObstacles>0
    obsStr = 'obstacle_';
else
    obsStr = 'noObstacle_';
end

if reset
    resetStr = 'reset_';
else
    resetStr = 'noReset_';
end

if cache
    cacheStr = 'cache_';
else
    cacheStr = 'noCache_';
end

if strcmp(entry,'default')
    entryStr = 'defaultEntry';
elseif strcmp(entry,'new')
    entryStr = 'newEntry';
else
    error('unrecognized entry type');
end

fileName = [simStr,blockStr,obsStr,resetStr,cacheStr,entryStr];
if strcmp(simType,'single')
    fileName = [fileName,speedStr,'.mat'];
else
    fileName = [fileName,'.mat'];
end

res = load(['sims/',fileName]);
agent = res.agent;
environment = res.environment;
trial = res.trial;
dateStr = res.dateStr;
if strcmp(simType,'single')
    results = res.singleAgentResults;
else
    results = res.multiAgentResults;
end
