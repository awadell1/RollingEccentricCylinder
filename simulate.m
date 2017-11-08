function [data, t, z] =simulate(p)
% Find IC for system
z0 = computeIC(p.omega0, p.R, p.gamma, p.theta0);

% ODE Options
tol = 1e-12;
optsRoll = odeset('RelTol', tol, 'AbsTol', tol, 'Events', @hasJumped);
opsBallistic = odeset('RelTol', tol, 'AbsTol', tol, 'Events', @hasLanded);


%% Simulate Cylinder
tFinal = 40;
[tR, zR] = ode45(@RHS_rolling, [0, tFinal], z0, optsRoll, p);

if (tR(end) < tFinal)
	tlift = tR(end); z0n = zR(end,:);
	z0n(2) = z0(2) -p.gamma*(z0n(1) - z0(1)); %Ensure tangency
	[tB, zB] = ode45(@RHS_ballistic, [tlift, tFinal], z0n, opsBallistic, p);
	
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