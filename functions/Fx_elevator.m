function F_x = Fx_elevator(elevator_pprz,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

angle = elevator_pprz2angle(elevator_pprz);

%Fx = (k1+k2*alpha+k3*alpha^2)*V^2
Fx_elevator_coeff = [-0.002608468213419 ...
                     -0.030215146812742 ...
                     -0.000271549151544];

F_x = (Fx_elevator_coeff(1) + ...
       Fx_elevator_coeff(2).*angle + ...
       Fx_elevator_coeff(3).*angle.^2).*V.^2;
end