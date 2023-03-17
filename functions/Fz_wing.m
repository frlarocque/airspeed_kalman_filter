function F_z = Fz_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fx1 = ((k1+k2*alpha+k3*alpha^2)*(sin(skew)^2+k4)*V^2
Fx_wing_coeff = [-0.064260709110356 ...
                 -1.047985886559624 ...
                 0.187628429983977 ...
                 0.518664673679778];

F_z = ((Fx_wing_coeff(1)+Fx_wing_coeff(2).*alpha+Fx_wing_coeff(3).*alpha.^2).*(sin(skew).^2+Fx_wing_coeff(4))).*V.^2;

end