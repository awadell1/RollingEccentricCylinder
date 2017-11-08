function [c1,c2] = constraint_violations(D1_x,D1_y,D1_theta,R,gamma)
%CONSTRAINT_VIOLATIONS
%    [C1,C2] = CONSTRAINT_VIOLATIONS(D1_X,D1_Y,D1_THETA,R,GAMMA)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    07-Nov-2017 22:24:35

t2 = gamma.^2;
t3 = t2+1.0;
t4 = 1.0./sqrt(t3);
c1 = D1_y.*t4+D1_x.*gamma.*t4;
if nargout > 1
    c2 = D1_x.*t4+D1_theta.*R-D1_y.*gamma.*t4;
end