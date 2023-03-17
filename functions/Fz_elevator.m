function F_z = Fz_elevator(elevator_pprz,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

angle = elevator_pprz2angle(elevator_pprz);

% Saturate at +- 15 deg for stall condition
angle = max(min(angle,deg2rad(15)),deg2rad(-15));

%Fz = (k1+k2*alpha+k3*alpha^2)*V^2
Fz_elevator_coeff = [0.001095783351808 ...
                     -0.214593908091671];

F_z = (Fz_elevator_coeff(1) + ...
       Fz_elevator_coeff(2).*angle).*V.^2;
end