function plotParams = loadPlotParams(plotType)
% LOADPLOTPARAMS Loads parameters that specify the trial protocol for a 
% simulated experiment.
%
%   plotParams = LOADPLOTPARAMS(plotType) takes as input a string that 
%   specifies the plotting configuration (currently configured for 
%   a white background using plotType='whiteBG').  

if strcmp(plotType,'whiteBG')
    plotParams.cFig      = [ 1, 1, 1];          % figure color
    plotParams.cArena    = [ 1, 1, 1];          % arena color
    plotParams.cTarget   = [.7,.7,.7];          % fill color for targets 
    plotParams.cObstacle = [.8, 0,.4];          % fill color for obstacles 
    plotParams.cAnchor   = [ 0, 0, 0];          % marker color for anchors
    plotParams.cTraj     = [ 0, 0, 0];          % line color for trajectory
    plotParams.cBoundary = [ 0, 0, 0];          % line color for boundary
    plotParams.cBlocks   = [[255,139,54];...    % color for plotting target blocks
                            [0,81,66];...
                            [128,20,85];...
                            [79,172,119];...
                            [255,127,140]]./255;
    plotParams.ms = 12;                         % marker size
    plotParams.fs = 16;                         % font size
    plotParams.lw = 2;                          % linewidth
    plotParams.windowSize = 4;                  % window used for averaging single-trial data

elseif strcmp(plotType,'blackBG')  
    plotParams.cFig      = [ 0, 0, 0];          % figure color
    plotParams.cArena    = [ 0, 0, 0];          % arena color
    plotParams.cTarget   = [.5,.5,.5];          % fill color for targets 
    plotParams.cObstacle = [.8, 0,.4];          % fill color for obstacles 
    plotParams.cAnchor   = [ 1, 1, 1];          % marker color for anchors
    plotParams.cTraj     = [ 1, 1, 1];          % line color for trajectory
    plotParams.cBoundary = [ 1, 1, 1];          % line color for boundary
    plotParams.cBlocks   = [[255,139,54];...    % color for plotting target blocks
                            [0,81,66];...
                            [128,20,85];...
                            [79,172,119];...
                            [255,127,140]]./255;
    plotParams.ms = 12;                         % marker size
    plotParams.fs = 16;                         % font size
    plotParams.lw = 2;                          % linewidth
    plotParams.windowSize = 4;                  % window used for averaging single-trial data
else
    error('unrecognized plot type');
end
