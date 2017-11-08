clear variables; close all
% Set Parameters
p.I= 15;
p.M= 20;
p.R= 1;
p.d= 0.4;
p.g= 10;
p.gamma = 1;
p.theta0 = 0;
p.omega0 = 0;
default = p;

%% Simulate Base Case
[sim1, t, z] = simulate(p);

%% Check for Energy Conservation
[Ek,Ep] = energy_con(sim1.dx, sim1.dy, sim1.dtheta, p.I, p.M, p.d, p.g, sim1.theta, sim1.y);
figure; box on; hold on;
plot(sim1.t, Ek, 'LineWidth', 2)
plot(sim1.t, Ep, 'LineWidth', 2)
plot(sim1.t, Ek+Ep, 'LineWidth', 2)
legend('E_k', 'E_p', 'E_t', 'Location', 'northwest');
xlabel('Time'); ylabel('Energy');
axis tight

%% Check for Constraint Violations
[c1,c2] = constraint_violations(sim1.dx, sim1.dy, sim1.dtheta, p.R, p.gamma);
figure; box on; hold on;
plot(sim1.t, c1, 'LineWidth', 2)
plot(sim1.t, c2, 'LineWidth', 2)
plot([sim1.tlift,sim1.tlift], [min([c1;c2]), max([c1;c2])], 'r-');
legend('No Penetration', 'No Slip', 'Lift Off', 'Location', 'northwest')
xlabel('Time'); ylabel('Velocity')
axis tight

%% Compute Constraint Force
if ~isnan(sim1.tlift)
	iLift = sum(t<sim1.tlift);
else
	iLift = length(sim1.t);
end
sim1.Fn = zeros(size(sim1.t)); sim1.Ft = sim1.Fn;
for i = iLift:-1:1
	[A,b] = eom_rolling(sim1.dtheta(i),p.I,p.M,p.R,p.d,p.g,p.gamma,sim1.theta(i));
	sol = A\b;
	sim1.Fn(i) = sol(4);	sim1.Ft(i) = sol(5);
end

% Compute Required Mu
sim1.mu = abs(sim1.Ft./sim1.Fn);

figure; box on; hold on; grid on;
plot(sim1.t, sim1.Fn, 'LineWidth', 2)
plot(sim1.t, sim1.Ft, 'LineWidth', 2)
legend('Normal Force', 'Friction Force', 'Location', 'northwest')
xlabel('Time'); ylabel('Force')
axis tight

figure; box on; hold on;
plot(sim1.t, sim1.mu, 'LineWidth', 2)
xlabel('Time'); ylabel('Friction Coefficent')
grid on;
axis tight; ylim([0,1.5])

%% Animate
animate(t,z,sim1,p);