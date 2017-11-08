function animate(t, z, p)
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

% Animate
tic;
while toc < t(end)
	% Compute current state
	zC = interp1(t, z, toc);
	
	cyl.Position(1) = zC(1) - p.R;
	cyl.Position(2) = zC(2) - p.R;
	
	com.XData = zC(1) + p.d*cos(zC(3));
	com.YData = zC(2) + p.d*sin(zC(3));
	
	% Update Slope
	x = [zC(1)-border,zC(1)+border];
	y = [zC(2)-border,zC(2)+border];
	slope.XData = x;
	slope.YData = -x*p.gamma;
	
	% Update Border
	xlim(ax, x);
	ylim(ax, y);
	
	drawnow limitrate
end
