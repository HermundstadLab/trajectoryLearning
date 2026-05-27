function r = trimResults(res,indRemove)

r = res;

r.trajectory.rewards(indRemove) = [];
r.trajectory.obstacleHits(indRemove) = [];
r.trajectory.boundaryFlag(indRemove) = [];

fields = {'executed','augmented','planned'};

for i=1:3
    r.trajectory.(fields{i}).nAnchors(indRemove)  = [];
    r.trajectory.(fields{i}).distance(indRemove)  = [];
    r.trajectory.(fields{i}).initAngle(indRemove) = [];
    for j=indRemove
        r.trajectory.(fields{i}).path{j} = {};
    end

    r.belief.target.likelihoods.(fields{i})(:,:,indRemove) = [];
end

r.trajectory.executed.latency(indRemove)  = [];

r.belief.target.posteriors(:,:,indRemove) = [];
r.belief.target.errormaps(:,:,indRemove)  = [];

r.belief.target.outcomeSurprise(indRemove)  = [];
r.belief.target.rewardProb(indRemove)       = [];
r.belief.target.posteriorEntropy(indRemove) = [];
r.belief.target.resetFlag(indRemove)        = [];
r.belief.target.cacheFlag(indRemove)        = [];

r.belief.context.allCaches(:,:,:,indRemove) = [];
r.belief.context.posteriors(:,indRemove)    = [];
r.belief.context.toRead(indRemove)   = [];
r.belief.context.toWrite(indRemove+1) = [];
r.belief.context.posteriorEntropy(indRemove) = [];

