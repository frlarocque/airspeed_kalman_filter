function F_x = Fx_elevator(elevator_cmd,V,isPPRZ)
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

%Fx = (k1+k2*alpha+k3*alpha^2)*V^2
Fx_elevator_coeff = [-2.60846821341900e-003 ...
                     -3.021514681274200e-02 ...
                     -0.000271549151544];

F_x = (Fx_elevator_coeff(1) + ...
       Fx_elevator_coeff(2).*angle + ...
       Fx_elevator_coeff(3).*angle.^2).*V.^2;
end