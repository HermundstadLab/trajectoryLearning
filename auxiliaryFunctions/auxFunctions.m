function auxFunctions()

end

function L = getError(x,y,xorig,yorig,Tnorm,Rnorm,tRange,rRange,sigma)

err = sqrt((x-xorig).^2+(y-yorig).^2);
err = err./max(err);
L = zeros(size(Rnorm));
inds = find(err>0);
for i=1:numel(inds)
    [R,Theta] = convertToPolar(x(inds(i)),y(inds(i)));
    lmap = gaussian(Tnorm,Rnorm,Theta./diff(tRange),R./diff(rRange),sigma,sigma);
    lmap(isnan(lmap)) = 0;
    L = L+lmap./100;
end

end

function [posterior,resetBelief] = updateBelief(prior,Loutcome,cacheSignal,mask)

np = size(prior,1);

% determine whether to cache
if numel(cacheSignal)>1 && cacheSignal(end)>1.75 && cacheSignal(end-1)>1.75
    % if cache signal drops below threshold, cache current posterior and revert to uniform posterior prior
    resetBelief = 1;
    posterior = normalizeBelief(mask.*ones(np,np)); 
else
    % else use prior and likelihood to update posterior
    resetBelief = 0;
    posterior = normalizeBelief(prior.*Loutcome); 
end

end



function [x,y] = adjustTrajectoryArena(x,y,xbounds,ybounds)
if any(abs(x)>max(xbounds))
    x(x<=xbounds(1)) = xbounds(1);
    x(x>=xbounds(2)) = xbounds(2);
end
if any(y>max(ybounds) | y<min(ybounds))
    y(y<=ybounds(1)) = ybounds(1);
    y(y>=ybounds(2)) = ybounds(2);
end
end

function [xnew,ynew] = adjustTrajectoryObstacle(x,y,X,Y,arenaMask,xBoundsObst,yBoundsObst,np)
%find indices of trajectory that pass through target
[~,inds] = min(abs(x-X(:)) + abs(y-Y(:)),[],1);
obstVec = arenaMask(inds);

%find timepoints when trajectory enters and exits obstacle
entrances = find(diff(obstVec)>0);
exits     = find(diff(obstVec)<0)+1;

invertBoundary = ...
    [[ 1,-1, 1];...
    [  1,-1,-1];...
    [  2, 1,-1];...
    [  2,-1,-1];...
    [  3, 1, 1];...
    [  3, 1,-1];...
    [  4, 1, 1];...
    [  4,-1, 1]];

xInsert = {};
yInsert = {};
for i=1:numel(entrances)
    %determine which boundary of obstacle the trajectory entered
    bins = [0,1,2,3,4]*np+1;

    xent = x(entrances(i));
    yent = y(entrances(i));
    [~,indStart] = min( abs(x(entrances(i))-xBoundsObst) + abs(y(entrances(i))-yBoundsObst) );

    indEdge = find(histcounts(indStart,bins));

    %determine direction of trajectory at entrance
    dirVec = [sign(diff(x(entrances(i)-1:entrances(i)))),sign(diff(y(entrances(i)-1:entrances(i))))];

    %shift boundary to align with entrance
    xtmp = circshift(xBoundsObst,numel(xBoundsObst)-indStart+1);
    ytmp = circshift(yBoundsObst,numel(yBoundsObst)-indStart+1);

    [~,indExit] = min( abs(x(exits(i))-xtmp) + abs(y(exits(i))-ytmp) );

    %for time interpolation
    tbase = linspace(0,1,numel(entrances(i):exits(i)));
    
    %determine if direction of obstacle boundary trajectory should be
    %inverted
    if ismember([indEdge,dirVec],invertBoundary,'rows')

        %time interpolation
        indObst  = indExit+1:numel(xtmp);
        tobst    = linspace(0,1,numel(indObst))';
        [~,indt] = min(abs(tbase-tobst),[],1);

        xInsert{i} = fliplr(xtmp(indObst(indt)));
        yInsert{i} = fliplr(ytmp(indObst(indt)));
    else

        %time interpolation
        indObst  = 1:indExit;
        tobst    = linspace(0,1,numel(indObst))';
        [~,indt] = min(abs(tbase-tobst),[],1);

        xInsert{i} = xtmp(indObst(indt));
        yInsert{i} = ytmp(indObst(indt));
    end

end

if numel(entrances)>0
    xnew = x(1:entrances(1)-1);
    ynew = y(1:entrances(1)-1);
    for i=1:numel(entrances)-1
        xnew = [xnew,xInsert{i},x(exits(i)+1:entrances(i+1)-1)];
        ynew = [ynew,yInsert{i},y(exits(i)+1:entrances(i+1)-1)];
    end
    xnew = [xnew,xInsert{end},x(exits(end)+1:numel(x))];
    ynew = [ynew,yInsert{end},y(exits(end)+1:numel(y))];
else
    xnew = x;
    ynew = y;
end


end

function [xBounds,yBounds] = getBounds(xRange,yRange,np)

xBounds = [linspace(xRange(1),xRange(2),np),...
    linspace(xRange(2),xRange(2),np),...
    linspace(xRange(2),xRange(1),np),...
    linspace(xRange(1),xRange(1),np)];

yBounds = [linspace(yRange(1),yRange(1),np),...
    linspace(yRange(1),yRange(2),np),...
    linspace(yRange(2),yRange(2),np),...
    linspace(yRange(2),yRange(1),np)];
end

function [xpad,ypad] = padTrajectory(x,y,nkmax,nInterp)
xpad = nan(1,(nkmax+1)*nInterp);      
ypad = nan(1,(nkmax+1)*nInterp);      
xpad(1:numel(x)) = x;              
ypad(1:numel(y)) = y; 
end

function mask = createPosteriorMask(axR,axTH,axX,axY,npExclude)

np = numel(axX);

% define boundaries of arena in X-Y coords and R-Theta coords
arenaX = [axX,max(axX).*ones(1,np),fliplr(axX),min(axX).*ones(1,np),axX(1)];
arenaY = [min(axY).*ones(1,np),axY,max(axY).*ones(1,np),fliplr(axY),min(axY)];
[arenaR,arenaTH] = convertToPolar(arenaX,arenaY);

% remove repeated values
[~,iiTH] = unique(arenaTH);
arenaR  = arenaR( iiTH);
arenaTH = arenaTH(iiTH);

% symmetrize amplitude
arenaR = (arenaR + fliplr(arenaR))./2;

%disallow very small amplitudes
mask = nan(np);
for i=1:np
    [~,ii] = min((axTH(i)-arenaTH).^2);
    [~,jj] = min((axR-arenaR(ii)).^2);
    mask(npExclude+1:jj,i) = 1;   
end

%trim boundaries
mask(end,:) = nan;
mask(:, 1 ) = nan;
mask(:,end) = nan;
end

function Poutcome = computeOutcomeProb(likelihood,outcomeIndex,prior,mask)
Poutcome = sum(sum(likelihood(:,:,outcomeIndex).*mask.*prior,'omitnan'),'omitnan');
end

function S = computeSurprise(P)
S = -log2(P);
end

function H = computeEntropy(P)
P = P(:);
P(isnan(P)) = [];
P(P<eps)=[];
H = sum(P.*computeSurprise(P));
end




