function [d,davg] = plotTrialAverages(xdat,ydat,plotParams,color,linestyle,yaxisLabel,smooth)

if nargin<7
    smooth = 1;
    if nargin<6
        yaxisLabel = '';
        if nargin<5
            linestyle = '-';
            if nargin<4
                color = 'k';
            end
        end
    end
end

% compute averages
if smooth
    d = movmean(ydat,plotParams.windowSize,1);
else
    d = ydat;
end
davg = mean(d,2);

hold on;plot(xdat,davg,linestyle,'color',color,'linewidth',plotParams.lw);
xlabel('trials');
ylabel(yaxisLabel)
set(gca,'fontsize',16)

end