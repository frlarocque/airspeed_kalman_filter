function F_x = Fx_hover_prop(hover_prop_RPM,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fx = K1*V^2+K2*RPM^2*sqrt(V)
Fx_hover_prop_coeff = [1.0.*-3.614335398771809E-3 ...
                       1.2.*-1.180349532783032E-7 ];

V = abs(V); %to protect agains the sqrt

F_x = Fx_hover_prop_coeff(1).*V.^2.*sign(V) + ...
     Fx_hover_prop_coeff(2).*sqrt(V).*sign(V).*hover_prop_RPM.^2;

% Alternative:
%Fx = K1*V^2+K2*RPM^2
%Fx hover props = -1.188293370185022E-2*V^2+- -3.133468238831906E-7*RPM^2
end