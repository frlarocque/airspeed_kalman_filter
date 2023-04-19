function F_x = Fx_pusher(pusher_prop_RPM,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% pwm2RPM =1.0e+04 *[0.001721000000000  -1.675885714285714] 

rho = 1.225; %kg/m^2
d = 9.*2.54/100; %m

J = V./((pusher_prop_RPM/60).*d); 

% CT = a*J+b
a = 1.0.*-0.2;
b = 0.105;

% Fx_pusher = k1*RPM^2+k2*RPM*V+k3*V [N] in x-direction
% Fx_pusher_coeff = [3.703544293799073e-07 -6.005961822003111e-05 0]; %without offset
% Fx_pusher_coeff = [3.680278471152158e-07 -4.391797235636297e-05 -8.651770063289748E-2]; % Based on initial wind tunnel data
% Fx_pusher_coeff = [3.96222948E-07 -5.2930351318E-05 -2.68843366027904E-01];
Fx_pusher_coeff = [b.*rho.*d.^4./60.^2 a.*d.^3./60 0];

if V>0
    F_x = Fx_pusher_coeff(1) .* pusher_prop_RPM.^2   +...
          Fx_pusher_coeff(2) .* pusher_prop_RPM .* V +...
          Fx_pusher_coeff(3) .* V;
else
   F_x = Fx_pusher_coeff(1) .* pusher_prop_RPM.^2;
end

CT = F_x./(rho*(pusher_prop_RPM/60).^2*d.^4);

end