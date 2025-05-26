function plotBelief(posterior,belief,plotParams,plotTitle)
% PLOTBELIEF Plot a single trajectory embedded in the arena.
%
%   PLOTBELIEF(posterior,belief,plotTitle) plots a quantity from the 
%   agent's belief (e.g. the prior, likelihood, or posterior). The input 
%   'belief' structure contains information for plotting the bounds of the 
%   belief. The input string 'plotTitle' specifies which quantity is being
%   plotted, and is used as the title of the plot.
%
%   See also: PLOTANCHORS, PLOTTRAJECTORY, PLOTCONTROLPARAMS,PLOTSINGLETRIAL

if nargin<4
    plotTitle = [];
end

% to facilitate comparison with cartesian coordinates, flip the posterior
% so that positive angles are defined clockwise from the -x axis, rather
% than counterclockwise from the +x axis.
imagesc(belief.thAxes,belief.rAxes,fliplr(posterior),'AlphaData',~isnan(posterior));
hold on; plot(belief.thBoundary,belief.rBoundary,'color',plotParams.cBoundary,'linewidth',plotParams.lw)

set(gca,'ydir','normal','fontsize',plotParams.fs)
title(plotTitle)
axis off

