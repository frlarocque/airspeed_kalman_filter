function [F_y] = Fy_wing(skew,alpha,u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%Fy = (K1+K2*cos(skew)*sin(skew)^2)*u^2
Fy_wing_coeff = [0E-2...
                 8.0E-2];

F_y = (Fy_wing_coeff(1)+Fy_wing_coeff(2).*cos(skew).^2.*sin(skew)).*u.*u.*sign(u);
end