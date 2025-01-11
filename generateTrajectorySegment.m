function [xtraj,ytraj,vtraj,htraj,d] = generateTrajectorySegment(t,dth,dr,phi,T)
% GENERATETRAJECTORYSEGMENT Generates a trajectory that, once appropriately
% offset, links a pair of anchor points.
%   [xtraj,ytraj,vtraj,htraj,d] = GENERATETRAJECTORYSEGMENT(t,dth,dr,phi,T) 
%   takes as input an ordered pair of anchor points that are defined in 
%   polar coordinates based on their angular separation 'dth' and their 
%   separation 'dr', an initial heading angle 'phi', and a total duration 
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
%   phi = initial heading offset, defined relative to dth-pi/2
%   T   = total time to travel between two anchors

% define auxiliary parameters
m     =  (  pi-2*phi)./T;
alpha =  (3*pi-2*phi)./T;
beta  = -(  pi+2*phi)./T;
omega =  (2*pi      )./T;
delta = phi+dth-pi/2;

% generate speed and heading over time
A = (-dr.*m.*alpha.*beta)./(2.*omega.^2.*cos(phi));
vtraj = A.*(1-cos(omega.*t));
htraj = m*t+delta;

% generate cartesian trajectory (closed-form solution to the integrals
% xtraj = cumsum(vtraj.*cos(htraj)) and ytraj = cumsum(vtraj.*sin(htraj))
xtraj = -dr.*(2.*omega.^2.*sin(delta)+alpha.*(2.*beta.*sin(delta+m.*t)-m.*sin(delta+beta.*t))-m.*beta.*sin(delta+alpha.*t))./(4.*omega.^2.*cos(phi));
ytraj =  dr.*(2.*omega.^2.*cos(delta)+alpha.*(2.*beta.*cos(delta+m.*t)-m.*cos(delta+beta.*t))-m.*beta.*cos(delta+alpha.*t))./(4.*omega.^2.*cos(phi));

% compute curvilinear distance along trajectory
d = abs(A.*T);

end
