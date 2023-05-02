function F_x = Fx_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fx2 = (k1+k5*skew+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
Fx_wing_coeff = [ 5.0.*-6.428638953316000e-03 ...
                  1.0.*1.671952644901720e-01 ...
                  1.0.*5.944103706458780e-01 ...
                  1.0.*3.983889380919000e-03 ...
                  1.0.*3.532085496834000e-03];

F_x = sign(V).*V.^2.*(Fx_wing_coeff(1)+Fx_wing_coeff(5).*sin(skew)+ (Fx_wing_coeff(2).*alpha+Fx_wing_coeff(3).*alpha.^2).*(sin(skew).^2+Fx_wing_coeff(4)));

end