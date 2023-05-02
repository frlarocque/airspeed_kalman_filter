function [F_y] = Fy_wing(skew,alpha,u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%Fy = (K1+K2*cos(skew)*sin(skew))*u^2
Fy_wing_coeff = [0E-2...
                 1E-3];

F_y = (Fy_wing_coeff(1)+Fy_wing_coeff(2).*cos(skew).*sin(skew).^2).*u.*u.*sign(u);
end