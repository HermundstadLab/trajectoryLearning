function plotParams = loadPlotParams(plotType)
% LOADPLOTPARAMS Loads parameters that specify the trial protocol for a 
% simulated experiment.
%
%   plotParams = LOADPLOTPARAMS(plotType) takes as input a string that 
%   specifies the plotting configuration (currently configured for 
%   a white background using plotType='white').  

if strcmp(plotType,'white')
    plotParams.cFig      = [ 1, 1, 1];      % figure color
    plotParams.cArena    = [ 1, 1, 1];      % arena color
    plotParams.cTarget   = [.7,.7,.7];      % fill color for targets 
    plotParams.cObstacle = [.8, 0,.4];      % fill color for obstacles 
    plotParams.cAnchor   = [.5, 0, 0];      % marker color for anchors
    plotParams.cTraj     = [.5, 0, 0];      % line color for trajectory
    plotParams.cBoundary = [ 0, 0, 0];      % line color for boundary
    plotParams.ms = 12;                     % marker size
    plotParams.fs = 16;                     % font size
    plotParams.lw = 2;                      % linewidth

%elseif strcmp(plotType,'black')            % uncomment to define plot
                                            %      parameters for black background
else
    error('unrecognized plot type');
end