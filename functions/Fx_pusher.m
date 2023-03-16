function F_x = Fx_pusher(pusher_prop_RPM,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% F_pusher = k1*RPM^2+k2*RPM*V+k3*V [N] in x-direction
% pusher_RPM2Fx_coeff = [3.703544293799073e-07 -6.005961822003111e-05 0]; %without offset
Fx_pusher_coeff = [3.680278471152158e-07 -4.391797235636297e-05 -8.651770063289748E-2];

F_x = Fx_pusher_coeff(1) .* pusher_prop_RPM.^2   +...
      Fx_pusher_coeff(2) .* pusher_prop_RPM .* V +...
      Fx_pusher_coeff(3) .* V;
end