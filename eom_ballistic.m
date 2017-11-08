function [A,b] = eom_ballistic(D1_theta,I,M,d,g,theta)
%EOM_BALLISTIC
%    [A,B] = EOM_BALLISTIC(D1_THETA,I,M,D,G,THETA)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    08-Nov-2017 12:43:47

t2 = cos(theta);
t3 = D1_theta.^2;
t4 = sin(theta);
A = reshape([M,0.0,0.0,0.0,M,0.0,-M.*d.*t4,M.*d.*t2,-I],[3,3]);
if nargout > 1
    b = [M.*d.*t2.*t3;-M.*(g-d.*t3.*t4);0.0];
end
