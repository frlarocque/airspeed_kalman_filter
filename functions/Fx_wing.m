function F_x = Fx_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fx2 = (k1*(1+k5*skew)+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
Fx_wing_coeff = [-0.007742841697518 ...
                 0.154451091730156 ...
                 0.704154476765070 ...
                 -0.043380923100371 ...
                 0.026848472271382];

F_x = V.^2.*(Fx_wing_coeff(1).*(1+Fx_wing_coeff(5).*skew)+ (Fx_wing_coeff(2).*alpha+Fx_wing_coeff(3).*alpha.^2).*(sin(skew).^2+Fx_wing_coeff(4)));

end