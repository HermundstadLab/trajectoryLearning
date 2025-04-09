function [xtraj,ytraj,vtraj,htraj,d] = generateTrajectorySegment(t,dth,dr,delta,planner)
% GENERATETRAJECTORYSEGMENT Generates a trajectory that, once appropriately
% offset, links a pair of anchor points.
%   [xtraj,ytraj,vtraj,htraj,d] = GENERATETRAJECTORYSEGMENT(t,dth,dr,delta,T) 
%   takes as input an ordered pair of anchor points that are defined in 
%   polar coordinates based on their angular separation 'dth' and their 
%   separation 'dr', an initial heading angle 'delta', and a total duration 
%   'T'. It returns the cartesian coordinates of a curvilinear trajectory 
%   ('xtraj' and 'ytraj'), together with the instantaneous heading ('htraj') 
%   and speed ('vtraj'). All outputs are defined over a time vector 't'.
%   This trajectory is defined in relative coordinates (based on the relative 
%   separation between anchor points); to position it in absolute 
%   coordinates, it must be offset by the [x,y] location of the first 
%   anchor point.
%
%   See also: PLANTRAJECTORY, RECOVERCONTROLPARAMS 

% variables in generative model:
%   t   = vector of timepoints
%   dth = angle between successive anchors
%   dr  = distance between successive anchors
%   delta = initial heading offset, defined relative to dth-pi/2
%   T   = total time to travel between two anchors

% define auxiliary parameters
gamma = delta./pi;
phi   = delta+dth;
omega = 2.*pi.*planner.rScale./dr;

% generate speed and heading over time
A = 2*planner.rScale.*(1-gamma.^2)./sinc(gamma);
vtraj = (A/2).*(1-cos(omega*t));
htraj = phi-omega.*gamma.*t;

% generate cartesian trajectory (closed-form solution to the integrals)
B = planner.rScale./sinc(gamma);

Cx = (1-gamma.^2).*(cos(phi).*sinc(omega.*gamma.*t/pi)+sin(phi).*sin(omega.*gamma.*t/2).*sinc(omega.*gamma.*t./(2*pi)));
Cy = (1-gamma.^2).*(sin(phi).*sinc(omega.*gamma.*t/pi)-cos(phi).*sin(omega.*gamma.*t/2).*sinc(omega.*gamma.*t./(2*pi)));

Dx =  1./(2.*omega).*(2.*gamma.*sin(phi)+(1-gamma).*sin(phi-omega.*(1+gamma).*t)-(1+gamma).*sin(phi+omega.*(1-gamma).*t));
Dy = -1./(2.*omega).*(2.*gamma.*cos(phi)+(1-gamma).*cos(phi-omega.*(1+gamma).*t)-(1+gamma).*cos(phi+omega.*(1-gamma).*t));

xtraj = B.*(Cx.*t + Dx);
ytraj = B.*(Cy.*t + Dy);

% compute curvilinear distance along trajectory
d = abs(A.*dr./(2.*planner.rScale));

end
