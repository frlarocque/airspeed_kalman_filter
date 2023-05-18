function F_x = Fx_fuselage(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fx = (k1*cos(skew)+k2+k3*alpha+k4*alpha^2)*V^2
Fx_fuselage_coeff = [1.1.*-8.111212221498999e-03 ...
                     1.6.*-2.477135327454600e-02 ...
                     0.0.*-8.297633291170999e-03 ...
                     0.0.*1.772463067231450e-01];

F_x = (Fx_fuselage_coeff(1)*cos(skew)+ ...
       Fx_fuselage_coeff(2) + ...
       Fx_fuselage_coeff(3).*alpha + ...
       Fx_fuselage_coeff(4).*alpha.^2).*V.^2;
end