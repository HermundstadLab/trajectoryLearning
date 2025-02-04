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
A = (-2.*dr.*alpha.*beta)./(omega.^2.*T.*sinc(m.*T/(2.*pi)));
vtraj = (A/2).*(1-cos(omega.*t));
htraj = m*t+delta;

% generate cartesian trajectory (closed-form solution to the integrals
B  = -dr./(omega.^2.*T.*sinc(m.*T/(2*pi)));
Cx =  alpha.*beta.*(cos(delta).*sinc(m.*t/pi)-sin(delta).*sin(m.*t./2).*sinc(m.*t./(2.*pi)));
Cy =  alpha.*beta.*(sin(delta).*sinc(m.*t/pi)+cos(delta).*sin(m.*t./2).*sinc(m.*t./(2.*pi)));

Dx =  m.*sin(delta) - (1./2).*(alpha.*sin(delta+beta.*t) + beta.*sin(delta+alpha.*t));
Dy = -m.*cos(delta) + (1./2).*(alpha.*cos(delta+beta.*t) + beta.*cos(delta+alpha.*t));

xtraj = B.*(Cx.*t + Dx);
ytraj = B.*(Cy.*t + Dy);

% compute curvilinear distance along trajectory
d = abs(A.*T./2);

end
