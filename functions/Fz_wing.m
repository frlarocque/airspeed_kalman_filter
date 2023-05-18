function F_z = Fz_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fz1 = ((k1+k2*alpha+k3*alpha^2)*(sin(skew)^2+k4)*V^2
%Fz1 = 
Fz_wing_coeff = [1.1.*-1.000778727574050e-01 ...
                 1.2.*-8.696479964371250e-01 ...
                 1.2.*1.457831456377660e-01 ...
                 0.0.*2.185394878246410e-01];

F_z = ((Fz_wing_coeff(1)+Fz_wing_coeff(2).*alpha+Fz_wing_coeff(3).*alpha.^2).*(sin(skew).^2+Fz_wing_coeff(4))).*V.^2;

end