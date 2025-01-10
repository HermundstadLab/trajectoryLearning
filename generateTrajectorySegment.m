function [xtraj,ytraj,vtraj,htraj] = generateTrajectorySegment(t,dth,dr,phi,T)
% GENERATETRAJECTORYSEGMENT Generates a trajectory that, once appropriately
% offset, links a pair of anchor points.
%   [xtraj,ytraj] = GENERATETRAJECTORYSEGMENT(t,dth,dr,phi,T) takes as input  
%   an ordered pair of anchor points that are defined in polar coordinates 
%   based on their angular separation 'dth' and their separation 'dr', 
%   an initial heading angle 'phi', and a total duration 'T'. It returns 
%   the cartesian coordinates of a curvilinear trajectory ('xtraj' and 
%   'ytraj'), together with the instantaneous heading ('htraj'), speed 
%   ('vtraj'), and speed amplitude ('A'). All outputs are defined over a 
%   time vector 't'. This trajectory is defined in relative coordinates 
%   (based on the relative separation between anchor points); to position 
%   it in absolute coordinates, it must be offset by the [x,y] location of 
%   the first anchor point, and scaled based on the temporal discretization 
%   of the trajectory (stored in planner.tScale).
%
%   See also: PLANTRAJECTORY, RECOVERCONTROLPARAMS 

% variables in generative model:
%   t   = vector of timepoints
%   dth = angle between successive anchors
%   dr  = distance between successive anchors
%   phi = initial heading offset, defined relative to dth-pi/2
%   T   = total time to travel between two anchors

% generate speed and heading over time
A = amplitude(dr,phi,T);
vtraj = velocity(t,A,T);
htraj = heading(t,phi,dth,T);

% integrate speed and heading to get position
xtraj = cumsum(vtraj.*cos(htraj));
ytraj = cumsum(vtraj.*sin(htraj));

end

function v = velocity(t,amplitude,T)
v = amplitude.*.5*(1-cos(t.*2.*pi./T));
end

function h = heading(t,phi,dth,T)
h = ((pi-2*phi).*t./T+phi+dth-pi/2);
end

function A = amplitude(dr,phi,T) 
A = (dr./T).*(pi-2*phi).*(pi+2*phi).*(3*pi-2*phi)./(4.*pi.^2.*cos(phi));
end
