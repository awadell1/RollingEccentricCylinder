function [newEq, newVar] = reduceOrder(eq, var)
assert(iscell(var));

newEq = formula(eq);
newVar = cell(numel(var)*3, 1); newVarI = 1;
for i = 1:length(var)
	dt = symvar(var{i}, 1);
	strVar = extractBefore(char(var{i}), '(');
	
	for dn = 2:-1:1
		newVar{newVarI} = sym(sprintf('D%d_%s', dn, strVar), 'real');
		newVarI = newVarI+1;
		
		newEq = subs(newEq, diff(var{i}, dt, dn), newVar{newVarI-1});
	end
	
	% Replace base function
	newVar{newVarI} = sym(strVar); newVarI = newVarI+1;
	newEq = subs(newEq, var{i}, newVar{newVarI-1});
end

newVar(newVarI:end) = [];
newVar = cell2sym(flip(reshape(newVar, 3, [])',2));