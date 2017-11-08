function data = sim2struct(t, z, term)
% Returns a struct containing all of the terms tracked by ode45
%
% INPUT
%	t		The time vector (dx1) returned by ode45
%	z		The state matrix (dxn) returned by ode45
%	term	A cell array of the name for each term tracked by ode45
%	Note: 
%		d - The number of time steps returned by ode45
%		n - The number of state variables tracked by ode45
% OUTPUT
%	data	A structure with fields for each term tracked by ode45 and the simulation time

	data.t = t;
	for i = 1:length(term)
		data.(term{i}) = z(:,i);
	end
end