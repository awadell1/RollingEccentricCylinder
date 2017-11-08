function animate(t, z, data, p)
figure; box on; hold on;
ax = gca;
axis equal

border = 5*p.R;

% Draw slope
slope = plot(0,0,'k', 'LineWidth', 2);

% Draw cylinder
cyl = rectangle('Position', [0 0 2*p.R 2*p.R], 'Curvature', [1 1]);
cyl.FaceColor = 'b';
cyl.EdgeColor = 'k';

% Mark COM
com = plot(0,0, 'ro', 'MarkerSize', 5);

% Draw Forces
frict = quiver(0,0,0,0,...
	1/(p.M*p.g),... % Scale Vector by force of gravity
	'g', 'LineWidth', 2);
gammaRad = atan(p.gamma);
en = [sin(gammaRad); cos(gammaRad); 0]; 	% Vector Normal to Slope Surface
et = cross(en, [0;0;1]);					% Vector Tangent to Slope Pointing Down

% Trace path
centerPath = plot(nan,nan,'k--');
comPath = plot(nan,nan, 'r--');

% Animate
tic;
rate = 0.2;
while toc*rate < t(end)
	% Compute current state
	tC = toc*rate;
	zC = interp1(t, z, tC);
	Fn = interp1(t, data.Fn, tC);
	Ft = interp1(t, data.Ft, tC);
	
	cyl.Position(1) = zC(1) - p.R;
	cyl.Position(2) = zC(2) - p.R;
	centerPath.XData = [centerPath.XData, zC(1)];
	centerPath.YData = [centerPath.YData, zC(2)];
	
	com.XData = zC(1) + p.d*cos(zC(3));
	com.YData = zC(2) + p.d*sin(zC(3));
	comPath.XData = [comPath.XData, com.XData];
	comPath.YData = [comPath.YData, com.YData];
	
	% Draw Friction Force
	frict.XData = zC(1) - z(1,1);
	frict.YData = zC(2) - z(1,2);
	F = [en, et] *[Fn; Ft];
	frict.UData = F(1);
	frict.VData = F(2);
	
	% Update Slope
	x = [zC(1)-border,zC(1)+border];
	y = [zC(2)-border,zC(2)+border];
	slope.XData = x;
	slope.YData = -x*p.gamma;
	
	% Update Border
	xlim(ax, x);
	ylim(ax, y);
	
	drawnow
end
