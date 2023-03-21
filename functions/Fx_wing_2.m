function F_x = Fx_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fx2 = (k1*(1+k5*skew)+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
Fx_wing_coeff = [-0.008484928911816 ...
                 0.153596528756635 ...
                 0.681068034638300 ...
                 -0.035001046966954 ...
                 0.001025145266924];

F_x = V.^2.*(Fx_wing_coeff(1)+Fx_wing_coeff(5).*sin(skew)+ (Fx_wing_coeff(2).*alpha+Fx_wing_coeff(3).*alpha.^2).*(sin(skew).^2+Fx_wing_coeff(4)));

end