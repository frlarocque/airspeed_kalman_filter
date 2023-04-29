function F_x = Fx_wing(skew,alpha,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Fx = (k1+k2*alpha+k3*alpha^2)*V^2
% k  = [k1 k2 k3 k4]
% x = [AoA,V]
% k = [0.052498667906677...
%      0.080193252948267...
%     -0.003364927732958];
% 
% F_x = (k(1)+k(2).*alpha+k(3).*alpha.^2).*V.^2;


% Fz = -L*cos(alpha) - D*sin(alpha)
% Fx = L*sin(alpha) -D*cos(alpha)

% Fx = (k1+k2*alpha*sin(skew)^2+k3*alpha^2*sin(skew)^2)*V^2
Fx_wing_coeff = [-0.0319-0.005768481331664...
                 0.40+0.006444411778767+0.006101013442505... %0.5
                 0.50+1.823432836610153e-04]; %0.5

% Fx7 = (k1+k2*alpha*sin(skew)^2+k3*alpha^2*sin(skew)^2)*V^2
%Fx_wing_coeff = [-1.0.*0.053390347807203...
%                 1.0.*0.077054107330026...
%                 1.0.*0.016842045233906];

F_x =  V.^2.*(Fx_wing_coeff(1)+Fx_wing_coeff(2).*alpha.*sin(skew).^2+Fx_wing_coeff(3).*alpha.^2.*sin(skew).^2);


%Fx = (k1*sin(skew)^2+k2*alpha*sin(skew)^2)
%Fx_wing_coeff = [2.5E-2 ...
%                 2E-3];

%F_x = V.^2.*(Fx_wing_coeff(1).*sin(skew).^2+Fx_wing_coeff(2).*alpha.*sin(skew).^2);

end