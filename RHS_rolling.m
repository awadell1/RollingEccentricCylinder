function dz = RHS_rolling(t, z, p)
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
D2_x = sol(1); D2_y = sol(2); D2_theta = sol(3);

dz = [D1_x; D1_y; D1_theta; D2_x; D2_y; D2_theta];
end