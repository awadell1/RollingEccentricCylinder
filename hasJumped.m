function [position,isterminal,direction] = hasJumped(t,z,p)
% Extract Parameters
I = p.I;
M = p.M;
R = p.R;
d = p.d;
g = p.g;
gamma = p.gamma;

% Extract State
x = z(1); y = z(2); theta = z(3);
D1_x = z(4); D1_y = z(5); D1_theta = z(6);

[A,b] = eom_rolling(D1_theta,I,M,R,d,g,gamma,theta);
sol = A\b;
Fn = sol(4);

position = Fn; % The value that we want to be zero
isterminal = 1;  % Halt integration 
direction = -1;   % The zero can be approached from either direction