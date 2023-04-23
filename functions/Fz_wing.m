function F_z = Fz_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

V = V.*cos(deg2rad(alpha));

% % From Tomaso's RW1 folder
% cl_max = 1.3;  % [-]                          
% S_wing = 1.56 * 0.235; % [m^2]
% rho = 1.225; % [kg/m^3]
% 
% % L = V^2*(K1*alpha*sin(skew)^2+K2*sin(skew)^2+K3*alpha+K4)
% %   = V^2*(sin(skew)^2*(K1*alpha+K2)+K3*alpha+K4)
% L = -0.5.*rho.*S_wing.*V.^2.*(-1.8847.*alpha.*sin(skew).^2 + ...
%                               1.1.*-0.2780.*sin(skew).^2+ ...
%                               -1.5037.*alpha + ...
%                               -0.0043.*1);
% 
% % Fz = -L*cos(alpha) - D*sin(alpha)
% % Fx = L*sin(alpha) -D*cos(alpha)
% 
% F_z = -L.*cos(alpha);


% Fx5 = (k1*sin(skew)^2     +alpha*(k2*sin(skew)  +k3)    +alpha^2*(k4*sin(skew)  +k5)   )*V^2
Fz_wing_coeff = [ 4.5*-0.017602202832124...
                  0.6*-0.771630045577992...
                  1.0*-0.208269413335601...
                  1.0*0.485369855639616...
                  1.0*-0.064413106842926];
F_z = V.^2.*(Fz_wing_coeff(1).*sin(skew).^2+alpha.*(Fz_wing_coeff(2).*sin(skew)+Fz_wing_coeff(3))+alpha.^2.*(Fz_wing_coeff(4).*sin(skew).^2+Fz_wing_coeff(5)));


end