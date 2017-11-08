function [data, t, z] =simulate(p)
% Vectors for slope calcs
gammaRad = atan(p.gamma);
en = [sin(gammaRad); cos(gammaRad); 0]; 	% Vector Normal to Slope Surface
CrelI = p.R*en;								% Center of Cylinder relative to Contact Point

% Start cylinder from rest with Contact point at the origin
x0 = CrelI(1);	y0 = CrelI(2); theta0 = p.theta0;
dx0 = 0; dy0 = 0; dtheta0 = 0;
z0 = [x0; y0; theta0; dx0; dy0; dtheta0];

% ODE Options
tol = 1e-8;
optsRoll = odeset('RelTol', tol, 'AbsTol', tol, 'Events', @hasJumped);
opsBallistic = odeset('RelTol', tol, 'AbsTol', tol, 'Events', @hasLanded);


%% Simulate Cylinder
tFinal = 40;
[tR, zR] = ode45(@RHS_rolling, [0, tFinal], z0, optsRoll, p);

if (tR(end) < tFinal)
	tlift = tR(end); z0 = zR(end,:); z0(2) = z0(2);
	[tB, zB] = ode45(@RHS_ballistic, [tlift, tFinal], z0, opsBallistic, p);
	
	% Combine Results
	t = [tR(1:end-1); tB]; z = [zR(1:end-1,:); zB];
else
	tlift = nan;
	t = tR; z = zR;
end


terms = {'x', 'y', 'theta', 'dx', 'dy', 'dtheta'};
data = sim2struct(t, z, terms);
data.tlift = tlift;
end