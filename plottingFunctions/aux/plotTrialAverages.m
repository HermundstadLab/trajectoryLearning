function [d,davg] = plotTrialAverages(xdat,ydat,plotParams,color,linestyle,yaxisLabel)

if nargin<6
    yaxisLabel = '';
    if nargin<5
        linestyle = '-';
        if nargin<4
            color = 'k';
        end
    end
end

[~,nAgents] = size(ydat);

% compute averages
d = movmean(ydat,[plotParams.windowSize,0],1);
davg = mean(d,2);
dste = std(d,[],2)./sqrt(nAgents);

% create vectors for patch objects
xvec = createPatchVec(xdat);
yvec = createPatchVec(davg',dste');

% plot results
h = fill(xvec,yvec,color,'edgecolor','none');
set(h,'facealpha',.5);

hold on;plot(xdat,davg,linestyle,'color',color,'linewidth',plotParams.lw);
xlabel('trials');
ylabel(yaxisLabel)
set(gca,'fontsize',16)

end

function patchVec = createPatchVec(vec,offset)
if nargin<2
    offset = zeros(size(vec));
end
patchVec = [vec-offset,fliplr(vec+offset),vec(1)-offset(1)];
end