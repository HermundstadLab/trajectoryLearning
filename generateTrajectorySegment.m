function [xtraj,ytraj,vtraj,htraj,d] = generateTrajectorySegment(t,dth,dr,delta,T)
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
m     =  (  pi-2*delta)./T;
alpha =  (3*pi-2*delta)./T;
beta  = -(  pi+2*delta)./T;
omega =  (2*pi      )./T;
phi   = delta+dth-pi/2;

% generate speed and heading over time
A = (-2.*dr.*alpha.*beta)./(omega.^2.*T.*sinc(m.*T/(2.*pi)));
vtraj = (A/2).*(1-cos(omega.*t));
htraj = m*t+phi;

% generate cartesian trajectory (closed-form solution to the integrals
B  = -dr./(omega.^2.*T.*sinc(m.*T/(2*pi)));
Cx =  alpha.*beta.*(cos(phi).*sinc(m.*t/pi)-sin(phi).*sin(m.*t./2).*sinc(m.*t./(2.*pi)));
Cy =  alpha.*beta.*(sin(phi).*sinc(m.*t/pi)+cos(phi).*sin(m.*t./2).*sinc(m.*t./(2.*pi)));

Dx =  m.*sin(phi) - (1./2).*(alpha.*sin(phi+beta.*t) + beta.*sin(phi+alpha.*t));
Dy = -m.*cos(phi) + (1./2).*(alpha.*cos(phi+beta.*t) + beta.*cos(phi+alpha.*t));

xtraj = B.*(Cx.*t + Dx);
ytraj = B.*(Cy.*t + Dy);

% compute curvilinear distance along trajectory
d = abs(A.*T./2);

end
