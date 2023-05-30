function F_x = Fx_wing_3(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Fx3 = (k1+k2*alpha+k3*alpha^2)*(sin(skew)^2+k4)*V^2
Fx_wing_coeff = [-0.004730248880130,...
                  0.182954145876165,...
                  0.670073870382007,...
                  1.0.*-0.072461249451492];
F_x = V.^2.*( (Fx_wing_coeff(1)+Fx_wing_coeff(2).*alpha+Fx_wing_coeff(3).*alpha.^2) .* (sin(skew).^2+Fx_wing_coeff(4)) );
end