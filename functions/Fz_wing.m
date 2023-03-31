function F_z = Fz_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fz1 = ((k1+k2*alpha+k3*alpha^2)*(sin(skew)^2+k4)*V^2
Fz_wing_coeff = [-0.100077872757405 ...
                 -0.869647996437125 ...
                  0.145783145637766 ...
                  0.218539487824641];

F_z = ((Fz_wing_coeff(1)+Fz_wing_coeff(2).*alpha+Fz_wing_coeff(3).*alpha.^2).*(sin(skew).^2+Fz_wing_coeff(4))).*V.^2;

end