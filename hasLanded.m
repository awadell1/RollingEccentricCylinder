function [position,isterminal,direction] = hasLanded(t,z,p)
% Extract Parameters
I = p.I;
M = p.M;
R = p.R;
d = p.d;
g = p.g;
gamma = -p.gamma;

% Extract State
x = z(1); y = z(2); theta = z(3);

% Distance to slope
dist = abs(gamma*x - y)/sqrt(1+gamma^2);

position = dist - R; % The value that we want to be zero
isterminal = 1;		% Halt integration 
direction = -1;		% The zero can be approached from either direction