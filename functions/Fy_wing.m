function F_y = Fy_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Fx = (k1+k2*alpha*sin(skew)^2+k3*alpha^2*sin(skew)^2)*V^2
Fy_wing_coeff = [0.5.*-0.063786655099948...
                 0.8.*1.456905472438850...
                 0.8.*7.171514141135752];

Fy_wing_coeff = [0.5.*-0.063786655099948...
                 0.8.*1.456905472438850...
                 0.8.*7.171514141135752];

F_y =  V.^2.*(Fy_wing_coeff(1)+Fy_wing_coeff(2).*alpha.*sin(skew).^2+Fy_wing_coeff(3).*alpha.^2.*sin(skew).^2);

F_y = 0.8.*-0.0288.*u.^2.*cos(skew).*sin(skew);

end