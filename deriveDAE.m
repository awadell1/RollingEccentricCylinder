clear variables
fprintf('Setting up system...')
syms x(t) y(t) theta(t)			% State Variables of the Cylinder
syms R M I d gamma g mu real	% Relevant Parameters
syms Fn Ft real					% Contact Forces

% Fixed Frame
ih = [1 0 0]'; jh = [0 1 0]'; kh = [0 0 1]';

% Slope Frame
gammaRad = atan(gamma);
en = [sin(gammaRad); cos(gammaRad); 0]; 	% Vector Normal to Slope Surface
et = cross(en, kh);							% Vector Tangent to Slope Pointing Down

% Cylinder Fixed Frame
er = [cos(theta); sin(theta); 0];	% Vector Rotating with Cylinder points at COM			

% Positions of Points on Cylinder
CrelO = [x;y;0];		% Center of Cylinder in Fixed Frame
IrelC = -R*en;			% Contact Point Relative to Center of Cylinder
GrelC = d*er;			% Center of Mass Relative to Center of Cylinder
GrelO = GrelC + CrelO;	% COM relative to Origin
IrelO = IrelC + CrelO;	% Contact Point relative to Origin
IrelG = IrelC - GrelC;	% Contact Point relative to COM

% Contact Force acting at C
F = Fn*en + Ft*et;

% LMB for Cylinder
LMB = M*diff(GrelO,2) == -M*g*jh + F;

% AMB about COM
AMB = cross(IrelG, F) == I*diff(theta,2)*kh;

%% Rolling Constraints
c1 = en'*diff(CrelO) == 0; % No Penetration
c1 = diff(c1);

c2 = diff(CrelO) == -R*diff(theta)*et; % No Slip
c2 = diff(c2);

fprintf('done\n')
%% Derive EOM for Rolling
fprintf('Finding Rolling EOM...')
% Replace diff terms with variables
EOM = [ih'*LMB; jh'*LMB; kh'*AMB; c1; c2];
[newEq, newVar] = reduceOrder(EOM, {x, y, theta});

% Solve for accelerations and constraint forces
ddState = [newVar(:,3); Fn; Ft];
[A, b] = equationsToMatrix(newEq, ddState);
A = simplify(A); b = simplify(b);

matlabFunction(A, b, 'File', 'eom_rolling');
fprintf('done\n')

%% Derive EOM for Ballistic
fprintf('Finding Ballistic EOM...')
% LMB and AMB for when ballistic
LMB = M*diff(GrelO,2) == -M*g*jh;
AMB = 0 == I*diff(theta,2)*kh;

% Replace diff terms with variables
EOM = [ih'*LMB; jh'*LMB; kh'*AMB];
vars = {x, y, theta};
[newEq, newVar] = reduceOrder(EOM, vars);

% Solve for accelerations
ddState = newVar(:,3);
[A, b] = equationsToMatrix(newEq, ddState);
A = simplify(A); b = simplify(b);

matlabFunction(A, b, 'File', 'eom_ballistic');
fprintf('done\n')
%% Compute IC from Wo
fprintf('Finding IC given initial omega...')
% Cylinder starts with the contact point at the origin with an initial
% angular velocity w0
C0 = -IrelC;
v0 = -R*diff(theta)*et;
z0 = [ih'*C0; jh'*C0; theta; ih'*v0; jh'*v0; diff(theta)];

z0 = reduceOrder(z0, {x,y,theta});
matlabFunction(z0, 'File', 'computeIC');
fprintf('done\n')

%% Conservation of Energy
fprintf('Finding system energy...')
v2 = sum(diff(GrelO).^2); 
Ek = 0.5*(M*v2 + I*diff(theta)^2);
Ep = M*g*(jh'*GrelO);

Ek = reduceOrder(Ek, {x,y,theta});
Ep = reduceOrder(Ep, {x,y,theta});

matlabFunction(Ek, Ep, 'File', 'energy_con');
fprintf('done\n')

%% Constraint Violations
fprintf('Checking constraint violations...')
c1 = en'*diff(CrelO);					% No Penetration
c2 = et'*diff(CrelO) + R*diff(theta);	% No Slip

c1 = reduceOrder(c1, {x,y,theta});
c2 = reduceOrder(c2, {x,y,theta});

matlabFunction(c1, c2, 'File', 'constraint_violations');
fprintf('done\n')
