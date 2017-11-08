function [A,b] = eom_rolling(D1_theta,I,M,R,d,g,gamma,theta)
%EOM_ROLLING
%    [A,B] = EOM_ROLLING(D1_THETA,I,M,R,D,G,GAMMA,THETA)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    07-Nov-2017 22:24:07

t2 = gamma.^2;
t3 = t2+1.0;
t4 = 1.0./sqrt(t3);
t5 = cos(theta);
t6 = sin(theta);
t7 = gamma.*t4;
t8 = R.*t4;
t9 = R.*gamma.*t4;
A = reshape([M,0.0,0.0,t7,1.0,0.0,0.0,0.0,M,0.0,t4,0.0,1.0,0.0,-M.*d.*t6,M.*d.*t5,-I,0.0,t8,-t9,0.0,-gamma.*t4,-t4,-d.*t4.*(t5-gamma.*t6),0.0,0.0,0.0,0.0,-t4,t7,t4.*(t8+d.*t6)+gamma.*t4.*(t9+d.*t5),0.0,0.0,0.0,0.0],[7,5]);
if nargout > 1
    t10 = D1_theta.^2;
    b = [M.*d.*t5.*t10;-M.*(g-d.*t6.*t10);0.0;0.0;0.0;0.0;0.0];
end