function F_z = Fz_elevator(elevator_cmd,V,isPPRZ)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if nargin<3
    angle = elevator_pprz2angle(elevator_cmd);
else
    if isPPRZ
        angle = elevator_pprz2angle(elevator_cmd);
    else
    angle = elevator_cmd;
    end
end

% Saturate at +- 15 deg for stall condition
angle = max(min(angle,deg2rad(15)),deg2rad(-15));

%Fz = (k1+k2*alpha+k3*alpha^2)*V^2
Fz_elevator_coeff = [0.001095783351808 ...
                     -0.214593908091671];

F_z = (Fz_elevator_coeff(1) + ...
       Fz_elevator_coeff(2).*angle).*V.^2;
end